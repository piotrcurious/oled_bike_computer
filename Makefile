CXX = g++
CXXFLAGS = -I. -Imock_arduino -Wall

SRCS = mock_arduino/Arduino.cpp
TEST_SRCS = test_ebike_simple.cpp test_speed_odo_trip.cpp test_speed_odo_trip_debounce.cpp
TEST_BINS = $(TEST_SRCS:.cpp=)

all: $(TEST_BINS)

test_ebike_simple: test_ebike_simple.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_speed_odo_trip: test_speed_odo_trip.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_speed_odo_trip_debounce: test_speed_odo_trip_debounce.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: all
	./test_ebike_simple
	./test_speed_odo_trip
	./test_speed_odo_trip_debounce

clean:
	rm -f $(TEST_BINS)

.PHONY: all test clean
