px4_add_module(
	MODULE modules__aatest
	MAIN aatest
	SRCS
		aaTest.cpp
		aaTest.hpp
	DEPENDS
		px4_work_queue
	)
