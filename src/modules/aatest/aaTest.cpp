#include "aaTest.hpp"



aaTest::aaTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}


bool aaTest::init()
{
	a = true;
	return true;
}


void aaTest::Run()
{
}

int aaTest::task_spawn(int argc, char *argv[])
{
	aaTest *instance = new aaTest();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int aaTest::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	PX4_INFO("a: %s", a ? "true" : "false");

	return 0;
}

int aaTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int aaTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int aatest_main(int argc, char *argv[]);

int aatest_main(int argc, char *argv[])
{
	return aaTest::main(argc, argv);
}
