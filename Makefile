CFLAGS := ${CFLAGS} -ggdb

ex1: examples/ex1.c
	${CC} ${CFLAGS} $< -L./build_linux/64/bin -littnotify -o $@
