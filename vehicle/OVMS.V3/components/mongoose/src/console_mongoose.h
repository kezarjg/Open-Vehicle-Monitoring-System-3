/*
;    Project:       Open Vehicle Monitor System
;    Date:          20th July 2026
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2026  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#ifndef __CONSOLE_MONGOOSE_H__
#define __CONSOLE_MONGOOSE_H__

#include <list>
#include "ovms_console.h"
#include "mongoose_client.h"

/**
 * ConsoleMongoose / ConsoleMongooseServer: shared teardown support for the
 * Mongoose based consoles (SSH, Telnet).
 *
 * The problem
 * -----------
 * An interactive command may run its own task that writes to the console that
 * launched it — "vfs tail" in follow mode is the current example. That task
 * holds a raw OvmsWriter pointer, so the console must not be freed while the
 * task can still write to it.
 *
 * OvmsWriter's termination handler registry solves that for consoles that can
 * simply join the task during teardown. A Mongoose based console cannot always
 * do that, because it is deleted from the MG_EV_CLOSE event, i.e. on the
 * Mongoose task, while the connection is being freed:
 *
 *  - On SSH the join deadlocks outright: MG_EV_CLOSE is dispatched with the
 *    Mongoose lock held for the whole mg_mgr_poll(), and the follow task's
 *    write() path needs that same lock, so neither side can proceed.
 *  - On Telnet the join does complete (its write() path calls mg_send()
 *    without taking the lock), but the follow task can still be inside
 *    mg_send() on the very connection Mongoose is about to free.
 *
 * The scheme
 * ----------
 * "Signal and reap" instead of "stop and wait". When a connection closes with
 * a follow-mode task still bound, the server marks the console closing (so the
 * task's in-flight write() bails out without touching the connection), asks the
 * task to stop without waiting for it, and defers the delete. A reap pass on a
 * later MG_EV_POLL frees the console once the task has actually exited.
 *
 * Consoles with no follow-mode task are deleted synchronously as before.
 */

class ConsoleMongoose : public OvmsConsole, public MongooseClient
  {
  public:
    ConsoleMongoose(struct mg_connection* nc);
    virtual ~ConsoleMongoose();

  public:
    // Set once the connection is closing. A write() from a follow-mode task
    // must check this FIRST and bail out: m_connection may already be freed,
    // so it must not be dereferenced afterwards.
    void SetClosing() { m_closing = true; }
    bool IsClosing() { return m_closing; }
    struct mg_connection* GetConnection() { return m_connection; }

  protected:
    struct mg_connection* m_connection;
    volatile bool m_closing;    // true once the connection is closing; read cross-task
  };

class ConsoleMongooseServer
  {
  public:
    virtual ~ConsoleMongooseServer();

  protected:
    // Call from the MG_EV_CLOSE handler in place of "delete child": deletes the
    // console immediately, or defers it if a follow-mode task is still bound.
    void CloseConsole(ConsoleMongoose* child);

    // Call from the MG_EV_POLL handler: frees any deferred console whose
    // follow-mode task has since exited.
    void ReapConsoles();

  private:
    std::list<ConsoleMongoose*> m_reaping;   // consoles awaiting follow-task exit before delete
  };

#endif //#ifndef __CONSOLE_MONGOOSE_H__
