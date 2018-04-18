#use g++/cpp so we can use glm
CC = /usr/bin/g++
#linux
#GLLIB := -lGL -lGLEW -lGLU -lglut
#mac
GLLIB := -framework OpenGL -framework GLUT
#windows (lookup a glut tutorial???)

#debug
CCFLAGS := -g
#Optomized
#note openmp might not work in os x
#CCFLAGS := -O3 -fopenmp -pthreads


INCDIR := -I/opt/local/include
LDLIBS := $(GLLIB) -L/opt/local/lib `libpng-config --ldflags`

TARGET = raycaster
OBJS = ray.o volraycaster.o glut_viewer.o  camera.o vrtransfer.o vrlight.o vrvolume.o

all: $(TARGET)

#stolen from joel porquet :-)
# Generate dependencies
DEPFLAGS = -MMD -MF $(@:.o=.d)

# Include dependencies
deps := $(patsubst %.o,%.d,$(OBJS))
-include $(deps)

$(TARGET): $(OBJS)
	$(CC)  $(LDLIBS) $^ -o $@

# Generic rule for compiling objects
%.o: %.c
	$(CC) $(CCFLAGS) $(INCDIR) -c -o $@ $< $(DEPFLAGS)

clean:
	rm -rf $(OBJS) $(TARGET)

