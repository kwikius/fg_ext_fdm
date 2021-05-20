

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
SRC_DIR = src
CXX = g++-7
CXXFLAGS = -fmax-errors=1 -std=c++14 -I$(QUAN_ROOT) -I$(SRC_DIR)/include
CXXLIBS = -lpthread

.PHONY : all test clean
all :  $(BIN_DIR)/main.exe 

$(BIN_DIR)/%.exe : $(BUILD_DIR)/%.o
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $< $(CXXLIBS)
	@echo .......................
	# executable in ./$@
	@echo ....... OK ............

$(BUILD_DIR)/%.o : $(SRC_DIR)/%.cpp 
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-rm -rf $(BUILD_DIR)/*.o $(BIN_DIR)/*.asm $(BIN_DIR)/*.exe


