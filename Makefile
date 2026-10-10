# ===============================
# Modbus RTU Master Makefile
# ===============================


CC = gcc


CFLAGS = -Wall -Wextra -O2 -Iinclude


TARGET = modbus_rtu


SRC_DIR = src
INC_DIR = include
BUILD_DIR = build


SRCS = $(wildcard $(SRC_DIR)/*.c)


OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))


# 默认目标
all: $(TARGET)



# 链接
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@



# 编译 .c -> build/*.o
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@



# 清理
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET)



# 查看
print:
	@echo $(SRCS)
	@echo $(OBJS)



.PHONY: all clean print
