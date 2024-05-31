// LIBRARIES = -framework OpenGL -framework GLUT -framework Carbon -lpng -framework OpenAL
LIBRARIES = -lpng -lGL -lGLU -lglut

EXE_DEBUG = ./bin/debug
EXE_TESTS = ./bin/tests
FILES_TO_LINKED = mapper.cpp bus.cpp mapper_000.cpp mapper_001.cpp mapper_002.cpp mapper_003.cpp mapper_004.cpp mapper_066.cpp cpu.cpp ppu.cpp apu.cpp cartridge.cpp

USE_ASLA = 1 clang++ -arch x86_64 -std=c++17 -Wall $(LIBRARIES) olcNes_Sounds1.cpp ${FILES_TO_LINKED} -o $(EXE_DEBUG) -v -shared -fPIC