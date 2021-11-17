CFLAGS = -g -DBOOST_TEST_DYN_LINK
LFLAGS = -lboost_coroutine -lboost_system -lboost_unit_test_framework
OBJS = test_dijkstra.o test_astar.o test.o


all: $(OBJS)
	g++ $(LFLAGS) -o test $(OBJS)

test: all
	./test

clean:
	rm -f *.o test

.cpp.o:
	g++ $(CFLAGS) -c $*.cpp

