# Makefile progetto Real Time - Venturini, Gentili

CC = gcc
CFLAGS = -Wall -g -O2
INC = -I.
ALLEG = `allegro-config --libs`
PTHREAD = -lpthread -lrt
MATH = -lm

.PHONY: all clean

all: bin/main

clean:
	@rm -f bin/* build/*
	@rmdir bin/ build/
	@echo "All claned...\n"

# Oggetti

build/main.o: src/main.c
	@mkdir -p build/
	@$(CC) -c $(INC) $< -o $@
	@echo "\n$@ created...\n"
build/%.o: src/%.c src/%.h
	@mkdir -p build/
	@$(CC) -c $(INC) $< -o $@
	@echo "\n$@ created...\n"

# Eseguibile

bin/main: build/main.o build/ptask.o build/task.o build/graphic.o build/matrix.o
	@mkdir -p bin/
	@$(CC) $^ -o $@ $(ALLEG) $(PTHREAD) $(MATH)
	@echo "\n$@ compiled...\n"
