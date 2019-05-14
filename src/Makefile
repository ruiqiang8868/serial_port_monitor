objects = fancy_cui.o serial_util.o

fancy_cui: $(objects)
	gcc -o fancy_cui fancy_cui.o serial_util.o

fancy_cui.o: fancy_cui.c
	gcc -c fancy_cui.c

serial_util.o: serial_util.c
	gcc -c serial_util.c

.PNONY:clean

clean:
	rm fancy_cui $(objects)
