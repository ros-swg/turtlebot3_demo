// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>

#include "rcutils/logging_macros.h"

/* Fork some threads to hog the CPU */
int start_hogs(uint forks)
{
  int pid, retval = 0;
  uint i;
  double d;

  for (i = 0; forks == 0 || i < forks; i++) {
    switch (pid = fork()) {
      case 0:  // If I'm the child fork
        // Use a backoff sleep to ensure we get good fork throughput.
        usleep(3000);

        // Do meaningless work!
        while (1) {
          d = sqrt(std::rand());
        }
      // This case never falls through, it works forever (until parent is exited)
      case -1:
        RCUTILS_LOG_ERROR("Worker fork failed.");
        return 1;
      default:
        // I'm the parent fork
        RCUTILS_LOG_INFO("--> Worker forked (%i)", pid);
    }
  }
  (void) d;  // For the linter

  // Wait for all child forks to exit
  while (i) {
    int status, ret;

    if ((pid = wait(&status)) > 0) {
      if ((WIFEXITED(status)) != 0) {
        if ((ret = WEXITSTATUS(status)) != 0) {
          RCUTILS_LOG_INFO("Worker process %i exited %i", pid, ret);
          retval += ret;
        } else {
          RCUTILS_LOG_INFO("<-- Worker exited (%i)", pid);
        }
      } else {
        RCUTILS_LOG_INFO("<-- Worker signalled (%i)", pid);
      }

      --i;
    } else {
      RCUTILS_LOG_INFO("wait() returned error: %s", strerror(errno));
      RCUTILS_LOG_ERROR("detected missing hogcpu worker children");
      ++retval;
      break;
    }
  }

  return retval;
}


int main(int /* argc */, char ** /* argv */)
{
  return start_hogs(4);
}
