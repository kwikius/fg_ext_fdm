

ifeq ($(QUAN_ROOT),)
define requires_quan_message
  Requires quan library.
  Download https://github.com/kwikius/quan-trunk/archive/refs/heads/master.zip
  unzip in <projectdirectory>
  export QUAN_ROOT = /home/my/path/to/quan-trunk in this terminal
  then re-run make
endef
$(error $(requires_quan_message))
endif

BUILD_DIR = build
BIN_DIR = bin
SRC_DIR = ../../src
CXX = g++-9
CXXFLAGS = -fmax-errors=1 -std=c++2a -fconcepts -I$(QUAN_ROOT) -I$(SRC_DIR)/include
CXXLIBS = -lpthread

OBJECTS = $(patsubst %.o, $(BUILD_DIR)/%.o, \
 straightnlevel.o \
 flight_mode.o \
 fgfs_fdm_in.o \
 fgfs_telnet.o \
 flight_controller.o \
 joystick_dimension.o \
 sensors.o \
 sl_controller.o \
 aircraft.o \
 get_P_torque.o \
 get_I_torque.o \
 get_D_torque.o \
)

TARGET = straightnlevel.exe
VPATH = $(SRC_DIR)

.PHONY : all test clean

all :  $(BIN_DIR)/$(TARGET) 

$(BIN_DIR)/$(TARGET) : $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -o $@ $(OBJECTS) $(CXXLIBS)
	@echo .......................
	# executable in ./$@
	@echo ....... OK ............

$(BUILD_DIR)/%.o : %.cpp 
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-rm -rf $(BUILD_DIR)/*.o $(BIN_DIR)/*.asm $(BIN_DIR)/*.exe


