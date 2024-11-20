# Compiler and flags
CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11
LDFLAGS = -lm -lwebots

# Directories
BUILD_DIR = build
SRC_DIR_SUPERVISOR = supervisor
SRC_DIR_EPUCK = e-puck
BIN_SUPERVISOR = $(BUILD_DIR)/supervisor
BIN_EPUCK = $(BUILD_DIR)/e-puck

# Source files
SRC_SUPERVISOR = $(wildcard $(SRC_DIR_SUPERVISOR)/*.cc)
SRC_EPUCK = $(wildcard $(SRC_DIR_EPUCK)/*.c)

# Object files
OBJ_SUPERVISOR = $(SRC_SUPERVISOR:$(SRC_DIR_SUPERVISOR)/%.cc=$(BUILD_DIR)/%.o)
OBJ_EPUCK = $(SRC_EPUCK:$(SRC_DIR_EPUCK)/%.c=$(BUILD_DIR)/%.o)

# Default target
all: $(BIN_SUPERVISOR) $(BIN_EPUCK)

# Build supervisor binary
$(BIN_SUPERVISOR): $(OBJ_SUPERVISOR)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(OBJ_SUPERVISOR) -o $@ $(LDFLAGS)

# Build e-puck binary
$(BIN_EPUCK): $(OBJ_EPUCK)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJ_EPUCK) -o $@ $(LDFLAGS)

# Compile supervisor source files
$(BUILD_DIR)/%.o: $(SRC_DIR_SUPERVISOR)/%.cc
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CFLAGS) -c $< -o $@

# Compile e-puck source files
$(BUILD_DIR)/%.o: $(SRC_DIR_EPUCK)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up
clean:
	@rm -rf $(BUILD_DIR)
	@echo "Clean finished."

.PHONY: all clean

