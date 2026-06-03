# Native (host) decode tests for the Toyota e-TNGA vehicle module.
# Compiles the real module .cpp against the shared mock framework and runs them.
# No ESP-IDF / toolchain / hardware required:  make test

CXX      ?= g++
HOST     := ../../../test/host

CXXFLAGS := -std=c++17 -Wall -Wextra -g \
            -I$(HOST) \
            -I$(HOST)/mock \
            -I../src \
            -include $(HOST)/mock/mock_ovms.hpp

# Mock framework + every module translation unit (the class is split across
# these files, so all are needed to link IncomingPollReply's call graph).
SRC := $(HOST)/mock/mock_ovms.cpp \
       ../src/vehicle_toyota_etnga.cpp \
       ../src/etnga_metrics.cpp \
       ../src/etnga_poll_processor.cpp \
       ../src/etnga_poll_states.cpp \
       ../src/etnga_can_processor.cpp \
       ../src/etnga_tpms.cpp \
       test_etnga_decode.cpp

all: test

test: $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@
	./$@

clean:
	rm -f test

.PHONY: all test clean
