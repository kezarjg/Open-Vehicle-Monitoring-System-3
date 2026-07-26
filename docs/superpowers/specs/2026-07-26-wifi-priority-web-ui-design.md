# WiFi priority networks — web UI (issue #165)

**Date:** 2026-07-26
**Issue:** kezarjg#165
**Branch:** `feature/wifi-priority-web-ui`
**Ships with:** the WiFi priority firmware feature (kezarjg PR #120, `ec8f350c`) as a single
upstream PR to openvehicles/Open-Vehicle-Monitoring-System-3.

## Problem

The WiFi priority networks feature can only be configured from the shell. `/cfg/wifi` has no
reference to `wifi.priority*`, so the list is invisible and un-editable from a browser —
including on the road, which is when it matters. On the 2026-07-20 road trip the module would
not associate with the travel-router hotspot and there was no way to inspect or fix the list
from the web UI.

Three conditions make the feature silently do nothing. None is surfaced anywhere:

1. **A listed SSID with no saved credential is skipped.** `SelectPriorityAP` requires a
   non-empty `wifi.ssid <ssid>` value; entries failing that are passed over without comment.
2. **While priority mode is on, only listed networks are joined.** A saved-but-unlisted
   network is never selected — `SelectPriorityAP` returns −1 for it.
3. **A fixed client SSID, or the wrong WiFi mode, disables the feature entirely.**
   `PriorityActive()` requires `m_sta_ssid.empty()` (scanning mode) *and* a mode of
   `CLIENT`/`APCLIENT`. If `auto/wifi.ssid.client` is set, or `auto/wifi.mode` is anything
   other than `client`/`apclient`, no amount of list editing has any effect. This one is the
   worst: it is not mentioned in the issue, its cause lives on a *different page*
   (`/cfg/autostart`), and `auto/wifi.mode` defaults to `ap` — so the out-of-the-box state is
   one where the feature silently does nothing.

## Scope

In scope: the editor (enable toggle, ranked list, interval) and all four failure-visibility
items — the three warnings above plus the mode check.

Explicitly **not** needed: surfacing the priority rank in the web WiFi status. This already
works. `web_cfg_status.cpp:196` renders the status by executing `wifi status` and dumping the
output into a self-updating `<samp class="monitor" data-updcmd="wifi status">`, and
`OutputStatus` has printed `Priority: rank N/M` since PR #120. Issue checkbox 4 requires zero
code.

## Design

### Placement and code structure

A new fieldset, "Wifi priority networks", on `/cfg/wifi`, immediately below the existing
"Wifi client networks" fieldset. Two new functions in `web_cfg_wifi.cpp`, mirroring the
existing pair:

| Function | Role |
|---|---|
| `OutputWifiPriority(p, c)` | Render the fieldset (GET, and re-render on POST error) |
| `UpdateWifiPriority(p, c, warn, error)` | Parse and persist the POST |

`HandleCfgWifi` gains one call in each branch. **`OutputWifiTable` and `UpdateWifiTable` are
not modified.** They are shared with the AP networks table; leaving them alone keeps AP
networks out of the diff and makes the change additive, which is materially easier to get
reviewed upstream.

This is why the design uses a separate fieldset rather than folding priority into the client
networks table. A single table with row-order-as-priority is the tidier end state in the
abstract, but it would require reworking those shared helpers, and it would make row position
meaningful in a table whose rows come out of a `std::map` (i.e. alphabetical, with no
meaningful order today).

### Row set

Rows are the **union** of:

- saved client SSIDs — the `wifi.ssid` param's instances, skipping keys ending
  `.ovms.staticip`, and
- SSIDs already present in `network/wifi.priority`.

Taking the union rather than just the saved networks is what surfaces orphans. An SSID on the
priority list that no longer has a saved credential must appear (flagged) rather than being
silently dropped from the form and then silently deleted on save.

Ordering: listed networks first, in priority order, then unlisted saved networks
alphabetically.

### Form encoding

A **single hidden field**, `cfg_priority`, carries the CSV of checked SSIDs in their current
display order. Inline page JS recomputes it on every check, uncheck, and ▲/▼ move. The server
reads that one field and writes it straight to `network/wifi.priority`.

The obvious alternative — per-row `prio_ssid_<n>` inputs following the existing table's
pattern — does not survive reordering: input `name` indices are fixed at render time, so
moving a `<tr>` does not change them, and every reorder would have to renumber every `name`
attribute. The hidden-CSV approach sidesteps that entirely and matches how the value is
actually stored in config.

Tradeoff: with JS broken, submitting saves the unchanged initial value — a safe no-op rather
than a corruption. The page already depends on JS for `addRow`/`delRow`, so this introduces no
new requirement.

All page JS is inline in `web_cfg_wifi.cpp`. Nothing is added to `ovms.js`, so no
`script.js`/`script.js.gz` regeneration is involved and the web-assets CI check is unaffected.

### Persistence

Follows the delete-on-default convention already used in this file for `wifi.sq.good`,
`wifi.ap2client.timeout`, etc.:

| Config key | Written when | Deleted when |
|---|---|---|
| `network/wifi.priority` | list non-empty | list empty |
| `network/wifi.priority.enable` | enabled | disabled (default) |
| `network/wifi.priority.interval` | != 60 | == 60 (default) |

### Warnings and errors

Computed server-side, on both GET and POST, and emitted through the page's existing
`warnlist`/`errorlist` mechanisms.

**Warnings** (saved anyway):

| Condition | Message |
|---|---|
| Listed SSID has no saved password | Row highlighted; names the SSID and states it will be skipped |
| `auto/wifi.ssid.client` set while enabled | Priority is inactive because a fixed client SSID is configured; links to `/cfg/autostart` |
| `auto/wifi.mode` not `client`/`apclient` while enabled | Priority is inactive in this mode; links to `/cfg/autostart`. Note the key accepts `ap`/`client`/`apclient`/`off` and **defaults to `ap`**, so an untouched module has the feature inert |
| Saved networks not on the list, while enabled | **One aggregated** warning naming them |

The saved-but-unlisted warning is deliberately aggregated into a single message. Emitting one
per network would produce eight warnings for a user with ten saved networks and two
prioritised, which would train the user to ignore the warning block.

**Errors** (block the save, form redisplayed):

| Condition | Rationale |
|---|---|
| Interval < 10 | The firmware clamps to 10 silently; the UI should say so rather than accept a value it won't honour |
| Enabled with zero networks checked | `PriorityActive()` requires a non-empty list, so this saves a configuration that cannot work |

### Firmware change

One small addition to `esp32wifi::OutputStatus`. The existing `Priority:` line is inside
`if (PriorityActive())`, so in precisely the situation a user would be debugging — priority
enabled but inert — the line disappears and the status says nothing at all.

Add an `else if (m_priority_enable)` branch printing why the feature is inactive: fixed SSID,
unsupported mode, or empty list. Roughly six lines. Because the web status page renders
`wifi status` verbatim, this fixes the CLI and the web UI at once.

## Testing

There is no host-side test suite; the firmware only compiles in CI.

1. **Build:** GitHub Actions `Firmware build` green on the branch.
2. **On-device**, bench module via the launcher flash path:
   - enable/disable round-trips and persists
   - reorder persists across a page reload and matches `config list network`
   - orphan warning fires for a listed SSID with no credential
   - autostart warning fires when `auto/wifi.ssid.client` is set
   - aggregated unlisted warning fires and names the right networks
   - interval < 10 is rejected; enabling with nothing checked is rejected
   - CLI↔UI agreement both directions: `config set network wifi.priority "a,b"` shows in the
     UI, and a UI save is visible in `config list network`
   - `wifi status` explains inactivity in each of the three inert cases

## Documentation

- `docs/source/userguide/wifi/client.rst` — the priority section from PR #120 exists; extend
  it to describe the web UI and the three inactivity conditions.
- `vehicle/OVMS.V3/changes.txt` — an entry for the feature already exists. **Amend it** to
  mention web configuration rather than adding a second entry; `changes.txt` is the single
  biggest upstream merge-conflict magnet and should be touched once.

## Out of scope

- Drag-and-drop reordering. ▲/▼ buttons only — no new JS dependency is permitted, and
  drag targets are poor on a phone, which is the road-trip use case.
- Editing credentials from the priority fieldset. Passwords stay in the client networks table;
  the priority list only references SSIDs.
- Any change to the priority selection algorithm itself.
