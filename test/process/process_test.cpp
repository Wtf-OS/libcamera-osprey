/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Process test
 */

#include <array>
#include <iostream>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/process.h"

#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

static volatile sig_atomic_t signalSeen = 0;
static void interrupted(int) { signalSeen = 1; }

class ProcessTestChild
{
public:
	int run(int status)
	{
		usleep(50000);

		return status;
	}
};

class ProcessTest : public Test
{
public:
	ProcessTest()
		: exitStatus_(Process::NotExited), exitCode_(-1)
	{
	}

protected:
	int run()
	{
		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		Timer timeout;

		int exitCode = 42;
		std::array args{ to_string(exitCode) };
		proc_.finished.connect(this, &ProcessTest::procFinished);

		/* Test that kill() on an unstarted process is safe. */
		proc_.kill();

		/* Test starting the process and retrieving the exit code. */
		int ret = proc_.start(self(), args);
		if (ret) {
			cerr << "failed to start process" << endl;
			return TestFail;
		}

		timeout.start(2000ms);
		while (timeout.isRunning() && exitStatus_ == Process::NotExited)
			dispatcher->processEvents();

		if (exitStatus_ != Process::NormalExit) {
			cerr << "process did not exit normally" << endl;
			return TestFail;
		}

		if (exitCode != exitCode_) {
			cerr << "exit code should be " << exitCode
			     << ", actual is " << exitCode_ << endl;
			return TestFail;
		}

		/* Synchronous waits must not reap a running child or lose notifications. */
		if (proc_.waitForExit(-1) != -EINVAL || proc_.waitForExit(0))
			return TestFail;
		if (proc_.start(self(), args) || proc_.exitStatus() != Process::NotExited ||
		    proc_.waitForExit(0) != -ETIMEDOUT || proc_.waitForExit(1) != -ETIMEDOUT)
			return TestFail;
		if (proc_.waitForExit(2000) || proc_.exitStatus() != Process::NormalExit ||
		    proc_.exitCode() != exitCode || proc_.waitForExit(0))
			return TestFail;

		if (proc_.start(self(), args))
			return TestFail;
		proc_.kill();
		if (proc_.waitForExit(2000) || proc_.exitStatus() != Process::SignalExit)
			return TestFail;

		/* A timeout keeps asynchronous monitoring usable. */
		exitStatus_ = Process::NotExited;
		if (proc_.start(self(), args) || proc_.waitForExit(1) != -ETIMEDOUT)
			return TestFail;
		timeout.start(2000ms);
		while (timeout.isRunning() && exitStatus_ == Process::NotExited)
			dispatcher->processEvents();
		if (exitStatus_ != Process::NormalExit || exitCode_ != exitCode)
			return TestFail;

		/* EINTR retries must preserve the original deadline. */
		if (proc_.start(self(), args))
			return TestFail;
		struct sigaction action = {}, previous;
		action.sa_handler = interrupted;
		sigemptyset(&action.sa_mask);
		if (sigaction(SIGALRM, &action, &previous))
			return TestFail;
		itimerval timer = {};
		timer.it_interval.tv_usec = timer.it_value.tv_usec = 1000;
		if (setitimer(ITIMER_REAL, &timer, nullptr)) {
			sigaction(SIGALRM, &previous, nullptr);
			return TestFail;
		}
		const auto begin = std::chrono::steady_clock::now();
		ret = proc_.waitForExit(10);
		const auto elapsed = std::chrono::steady_clock::now() - begin;
		timer = {};
		setitimer(ITIMER_REAL, &timer, nullptr);
		sigaction(SIGALRM, &previous, nullptr);
		proc_.kill();
		if (ret != -ETIMEDOUT || !signalSeen || elapsed > 200ms || proc_.waitForExit(2000))
			return TestFail;

		return TestPass;
	}

private:
	void procFinished(enum Process::ExitStatus exitStatus, int exitCode)
	{
		exitStatus_ = exitStatus;
		exitCode_ = exitCode;
	}

	Process proc_;
	enum Process::ExitStatus exitStatus_;
	int exitCode_;
};

/*
 * Can't use TEST_REGISTER() as single binary needs to act as both
 * parent and child processes.
 */
int main(int argc, char **argv)
{
	if (argc == 2) {
		int status = std::stoi(argv[1]);
		ProcessTestChild child;
		return child.run(status);
	}

	ProcessTest test;
	test.setArgs(argc, argv);
	return test.execute();
}
