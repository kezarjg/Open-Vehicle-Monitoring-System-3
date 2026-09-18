#!/usr/bin/env python3
"""Upload files to a MinIO/S3 bucket with plain AWS SigV4-signed PUTs.

Standard library only, so CI needs no client download (dl.min.io stopped
serving the mc client) and does not depend on the runner's curl version
(curl < 7.86 mis-signs --aws-sigv4 uploads).

Usage: minio_put.py <endpoint> <bucket/prefix> <file>...
Credentials come from MINIO_ACCESS_KEY / MINIO_SECRET_KEY.
Each file is stored as <bucket/prefix>/<basename>.
"""

import datetime
import hashlib
import hmac
import os
import sys
import urllib.error
import urllib.parse
import urllib.request

REGION = "us-east-1"  # MinIO default region
SERVICE = "s3"


def _hmac(key, msg):
    return hmac.new(key, msg.encode(), hashlib.sha256).digest()


def put(endpoint, key_path, data, access_key, secret_key):
    url = urllib.parse.urlsplit(endpoint)
    host = url.netloc
    path = "/" + urllib.parse.quote(key_path.strip("/"), safe="/-_.~")
    now = datetime.datetime.utcnow()
    amz_date = now.strftime("%Y%m%dT%H%M%SZ")
    date = now.strftime("%Y%m%d")
    payload_hash = hashlib.sha256(data).hexdigest()

    signed_headers = "host;x-amz-content-sha256;x-amz-date"
    canonical_request = "\n".join([
        "PUT", path, "",
        "host:" + host,
        "x-amz-content-sha256:" + payload_hash,
        "x-amz-date:" + amz_date,
        "",
        signed_headers,
        payload_hash,
    ])
    scope = "/".join([date, REGION, SERVICE, "aws4_request"])
    string_to_sign = "\n".join([
        "AWS4-HMAC-SHA256", amz_date, scope,
        hashlib.sha256(canonical_request.encode()).hexdigest(),
    ])
    key = _hmac(("AWS4" + secret_key).encode(), date)
    key = _hmac(key, REGION)
    key = _hmac(key, SERVICE)
    key = _hmac(key, "aws4_request")
    signature = hmac.new(key, string_to_sign.encode(), hashlib.sha256).hexdigest()

    req = urllib.request.Request(
        url.scheme + "://" + host + path, data=data, method="PUT")
    req.add_header("x-amz-date", amz_date)
    req.add_header("x-amz-content-sha256", payload_hash)
    req.add_header("Authorization",
                   "AWS4-HMAC-SHA256 Credential=%s/%s, SignedHeaders=%s, Signature=%s"
                   % (access_key, scope, signed_headers, signature))
    with urllib.request.urlopen(req, timeout=120) as resp:
        return resp.status


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    endpoint, prefix, files = sys.argv[1], sys.argv[2], sys.argv[3:]
    access_key = os.environ["MINIO_ACCESS_KEY"]
    secret_key = os.environ["MINIO_SECRET_KEY"]
    for f in files:
        with open(f, "rb") as fh:
            data = fh.read()
        key_path = prefix.rstrip("/") + "/" + os.path.basename(f)
        print("Uploading %s (%d bytes) -> %s" % (f, len(data), key_path), flush=True)
        try:
            put(endpoint, key_path, data, access_key, secret_key)
        except urllib.error.HTTPError as e:
            sys.exit("Upload failed: HTTP %d\n%s" % (e.code, e.read().decode(errors="replace")))


if __name__ == "__main__":
    main()
