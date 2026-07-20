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

#include "console_mongoose.h"
#include "ovms_command.h"

//-----------------------------------------------------------------------------
//    Class ConsoleMongoose
//-----------------------------------------------------------------------------

ConsoleMongoose::ConsoleMongoose(struct mg_connection* nc)
  {
  m_connection = nc;
  m_closing = false;
  }

ConsoleMongoose::~ConsoleMongoose()
  {
  // Note: RunTerminationCallback() is deliberately NOT called here. It must run
  // before the derived console frees the transport its writer feeds, and a base
  // destructor runs after the derived one. Each console therefore calls it as
  // the first statement of its own destructor.
  }

//-----------------------------------------------------------------------------
//    Class ConsoleMongooseServer
//-----------------------------------------------------------------------------

ConsoleMongooseServer::~ConsoleMongooseServer()
  {
  // The console servers are global singletons living for the lifetime of the
  // firmware, so this is not reached in practice. Any still-deferred console is
  // deliberately leaked rather than freed: its follow-mode task may not have
  // exited yet, and freeing it would be a use-after-free.
  }

void ConsoleMongooseServer::CloseConsole(ConsoleMongoose* child)
  {
  OvmsCommandTask* ft = child->GetFollowModeTask();
  if (!ft)
    {
    delete child;   // no follow task: synchronous termination + free
    return;
    }

  // A follow-mode task is still bound to this console. We must not join it
  // here: we are on the Mongoose task with the lock held, and the connection is
  // being freed underneath us. Mark closing so the task's in-flight write()
  // bails out without touching the connection, ask it to stop, and defer the
  // delete to a later poll.
  child->SetClosing();
  ft->RequestStop();
  m_reaping.push_back(child);
  }

void ConsoleMongooseServer::ReapConsoles()
  {
  for (auto it = m_reaping.begin(); it != m_reaping.end(); )
    {
    ConsoleMongoose* z = *it;
    // GetFollowModeTask() turns NULL once the task has deregistered itself,
    // which ~OvmsCommandTask does as its last access to the writer.
    if (z->GetFollowModeTask() == NULL)
      {
      delete z;
      it = m_reaping.erase(it);
      }
    else
      ++it;
    }
  }

// (CI retrigger: telnet A-B-A build)
