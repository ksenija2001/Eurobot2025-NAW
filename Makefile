TARGET := server

SRC_DIR		:= src
INC_DIR		:= include
BUILD_DIR	:= build

CC			:= gcc
CFLAGS	 	:= -Wall -Wextra -Werror -std=c11 -I$(INC_DIR)

LDFLAGS := 

SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))

all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link executable
$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean build files
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: all clean
