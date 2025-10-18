## 디바이스 드라이버 커널 스텝모터(STEP MOTOR 28BYJ-48)

# 디바이스 드라이버
```c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#define DEVICE_NAME "stepper_motor"
#define MAJOR_NUM 230

// GPIO 핀 정의 (라즈베리파이 BCM 핀 번호)
#define IN1_PIN 530  // GPIO 530 (Physical Pin 12)
#define IN2_PIN 531   // GPIO 19 (Physical Pin 35)
#define IN3_PIN 532   // GPIO 20 (Physical Pin 38)
#define IN4_PIN 533   // GPIO 21 (Physical Pin 40)

// 모터 상수 정의
#define MAX_STEPS_PER_REVOLUTION 4098  // 28BYJ-48 Half-step 모드 2048이 반 바퀴
#define MIN_PERCENTAGE 0
#define MAX_PERCENTAGE 100

// 28BYJ-48 스텝모터의 8스텝 시퀀스 (Half-step mode)
static const int step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// IOCTL 명령 정의
#define STEPPER_MAGIC 's'
#define STEPPER_STEP _IOW(STEPPER_MAGIC, 1, int)
#define STEPPER_ROTATE _IOW(STEPPER_MAGIC, 2, int)
#define STEPPER_STOP _IO(STEPPER_MAGIC, 3)
#define STEPPER_SET_SPEED _IOW(STEPPER_MAGIC, 4, int)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stepper Motor Driver");
MODULE_DESCRIPTION("28BYJ-48 Stepper Motor Driver for Raspberry Pi 4");
MODULE_VERSION("1.0");

static DEFINE_MUTEX(stepper_mutex);

// 모터 상태 변수
static int current_step = 0;
static int motor_speed = 2; // ms 단위
static int is_running = 0;
static int current_position = 0; // 현재 위치 (0~100%)

// GPIO 핀 배열
static int gpio_pins[4] = {IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN};

// GPIO 초기화 함수
static int gpio_stepper_init(void) {
    int i, ret;
    
    printk(KERN_INFO "Initializing GPIO pins for stepper motor\n");
    
    for (i = 0; i < 4; i++) {
        ret = gpio_request(gpio_pins[i], "stepper_motor");
        if (ret) {
            printk(KERN_ERR "Failed to request GPIO %d\n", gpio_pins[i]);
            return ret;
        }
        gpio_direction_output(gpio_pins[i], 0);
    }
    
    printk(KERN_INFO "GPIO pins initialized successfully\n");
    return 0;
}

// GPIO 정리 함수
static void gpio_stepper_cleanup(void) {
    int i;
    
    printk(KERN_INFO "Cleaning up GPIO pins\n");
    
    for (i = 0; i < 4; i++) {
        gpio_free(gpio_pins[i]);
    }
}

// 스텝모터 제어 함수들
static void set_motor_pins(int step) {
    int i;
    for (i = 0; i < 4; i++) {
        gpio_set_value(gpio_pins[i], step_sequence[step][i]);
    }
}

static void step_motor(int direction) {
    if (direction > 0) {
        current_step = (current_step + 1) % 8;
    } else {
        current_step = (current_step - 1 + 8) % 8;
    }
    set_motor_pins(current_step);
}

static void stop_motor(void) {
    int i;
    for (i = 0; i < 4; i++) {
        gpio_set_value(gpio_pins[i], 0);
    }
    is_running = 0;
}

// 디바이스 파일 오픈
static int stepper_open(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "Stepper motor device opened\n");
    return 0;
}

// 디바이스 파일 닫기
static int stepper_release(struct inode *inodep, struct file *filep) {
    mutex_lock(&stepper_mutex);
    stop_motor();
    mutex_unlock(&stepper_mutex);
    printk(KERN_INFO "Stepper motor device closed\n");
    return 0;
}

// IOCTL 처리
static long stepper_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
    int retval = 0;
    int percentage;  // 사용자 앱에서 받는 퍼센트 값
    int steps;
    int direction;
    int speed;
    int movement;
    int i;
    
    mutex_lock(&stepper_mutex);
    
    switch (cmd) {
        case STEPPER_STEP:
            if (copy_from_user(&percentage, (int*)arg, sizeof(int))) {
                retval = -EFAULT;
                break;
            }
            
            if (percentage >= 0 && percentage <= 100) {
                // 상대적 이동량 계산
                movement = percentage - current_position;
                
                if (movement == 0) {
                    printk(KERN_INFO "Already at position %d%%, no movement needed\n", percentage);
                } else {
                    // 방향 결정
                    if (movement > 0) {
                        direction = 1;  // 정방향
                    } else {
                        direction = -1; // 역방향
                        movement = -movement; // 절댓값
                    }
                    
                    // 이동량을 스텝으로 변환
                    steps = (movement * MAX_STEPS_PER_REVOLUTION) / 100;
                    
                    printk(KERN_INFO "Moving from %d%% to %d%% (%d%% movement, %d steps) in direction %d\n", 
                           current_position, percentage, movement, steps, direction);
                    
                    // 스텝 실행
                    for (i = 0; i < steps; i++) {
                        step_motor(direction);
                        msleep(motor_speed);
                    }
                    
                    // 현재 위치 업데이트
                    current_position = percentage;
                    
                    printk(KERN_INFO "Completed movement to %d%%\n", current_position);
                }
            } else {
                retval = -EINVAL;
                printk(KERN_ERR "Invalid percentage value: %d (range: 0~100)\n", percentage);
            }
            break;
            
        case STEPPER_ROTATE:
            if (copy_from_user(&direction, (int*)arg, sizeof(int))) {
                retval = -EFAULT;
                break;
            }
            is_running = 1;
            printk(KERN_INFO "Starting continuous rotation\n");
            while (is_running) {
                step_motor(direction);
                msleep(motor_speed);
            }
            printk(KERN_INFO "Stopped continuous rotation\n");
            break;
        
        case STEPPER_STOP:
            stop_motor();
            printk(KERN_INFO "Motor stopped by user\n");
            break;
            
        case STEPPER_SET_SPEED:
            if (copy_from_user(&speed, (int*)arg, sizeof(int))) {
                retval = -EFAULT;
                break;
            }
            if (speed > 0 && speed <= 100) {
                motor_speed = speed;
                printk(KERN_INFO "Motor speed set to %dms\n", speed);
            } else {
                retval = -EINVAL;
                printk(KERN_ERR "Invalid speed value: %d\n", speed);
            }
            break;
            
        default:
            retval = -ENOTTY;
            printk(KERN_ERR "Unknown ioctl command: %d\n", cmd);
            break;
    }
    
    mutex_unlock(&stepper_mutex);
    return retval;
}

// 파일 오퍼레이션 구조체
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = stepper_open,
    .release = stepper_release,
    .unlocked_ioctl = stepper_ioctl,
};

// 모듈 초기화
static int __init stepper_init(void) {
    int result;
    
    printk(KERN_INFO "call stepper_init\n");
    
    // 문자 디바이스 등록
    result = register_chrdev(MAJOR_NUM, DEVICE_NAME, &fops);
    if (result < 0) {
        printk(KERN_ERR "Failed to register character device\n");
        return result;
    }
    
    // GPIO 초기화
    result = gpio_stepper_init();
    if (result < 0) {
        printk(KERN_ERR "Failed to initialize GPIO\n");
        unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
        return result;
    }
    
    printk(KERN_INFO "Stepper Motor Driver loaded with major number %d\n", MAJOR_NUM);
    printk(KERN_INFO "Device name: %s\n", DEVICE_NAME);
    printk(KERN_INFO "Create device file: mknod /dev/%s c %d 0\n", DEVICE_NAME, MAJOR_NUM);
    
    return 0;
}

// 모듈 정리
static void __exit stepper_exit(void) {
    printk(KERN_INFO "call stepper_exit\n");
    
    // 모터 정지
    stop_motor();
    
    // GPIO 정리
    gpio_stepper_cleanup();
    
    // 문자 디바이스 등록 해제
    unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
    
    printk(KERN_INFO "Stepper Motor Driver unloaded\n");
}

module_init(stepper_init);
module_exit(stepper_exit);
```




# 사용자 앱
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

// 디바이스 드라이버와 동일한 IOCTL 명령 정의
#define STEPPER_MAGIC 's'
#define STEPPER_STEP _IOW(STEPPER_MAGIC, 1, int)
#define STEPPER_STOP _IO(STEPPER_MAGIC, 3)

#define DEVICE_PATH "/dev/stepper_motor"

// 함수 선언
int open_device(void);
void close_device(int fd);
void send_rotation_value(int fd, int percentage);
void stop_motor(int fd);
void show_help(void);
void motor_control_mode(int fd);

int main(int argc, char *argv[]) {
    int fd;
    int mode_choice;
    char input[256];
    
    printf("=== 28BYJ-48 스텝모터 제어 프로그램 ===\n");
    printf("라즈베리파이4 + ULN2003 드라이버\n\n");
    
    // 디바이스 열기
    fd = open_device();
    if (fd < 0) {
        printf("디바이스를 열 수 없습니다. 드라이버가 로드되었는지 확인하세요.\n");
        printf("사용법: sudo insmod stepper_motor_driver.ko\n");
        return -1;
    }
    
    while (1) {
        printf("=== 모드 선택 ===\n");
        printf("1. 모터 제어 (0~100%%)\n");
        printf("2. 도움말\n");
        printf("3. 종료\n");
        printf("선택하세요: ");
        
        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }
        
        if (sscanf(input, "%d", &mode_choice) != 1) {
            printf("잘못된 입력입니다.\n\n");
            continue;
        }
        
        switch (mode_choice) {
            case 1:
                motor_control_mode(fd);
                break;
                
            case 2:
                show_help();
                break;
                
            case 3:
                printf("프로그램을 종료합니다.\n");
                close_device(fd);
                return 0;
                
            default:
                printf("잘못된 선택입니다.\n");
                break;
        }
        
        printf("\n");
    }
    
    close_device(fd);
    return 0;
}

// 모터 제어 모드 (UI만 담당)
void motor_control_mode(int fd) {
    int percentage;
    char input[256];
    
    while (1) {
        printf("\n=== 모터 제어 ===\n");
        printf("회전 값을 입력하세요 (0~100%%):\n");
        printf("  1~100: 정방향 회전 (예: 100 = 1회전 정방향)\n");
        printf("  0: 역방향 1회전\n");
        printf("  'q': 메인 메뉴로 돌아가기\n");
        printf("입력: ");
        
        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }
        
        // 'q' 입력 시 메인 메뉴로 돌아가기
        if (input[0] == 'q' || input[0] == 'Q') {
            return;
        }
        
        if (sscanf(input, "%d", &percentage)) {
            if (percentage >= 0 && percentage <= 100) {
                send_rotation_value(fd, percentage);
            } else {
                printf("0~100 범위의 값을 입력하세요.\n");
            }
        } else {
            printf("잘못된 입력입니다. 숫자 또는 'q'를 입력하세요.\n");
        }
    }
}

int open_device(void) {
    int fd;
    
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("디바이스 열기 실패");
        return -1;
    }
    
    printf("디바이스가 성공적으로 열렸습니다.\n");
    return fd;
}

void close_device(int fd) {
    if (close(fd) < 0) {
        perror("디바이스 닫기 실패");
    } else {
        printf("디바이스가 닫혔습니다.\n");
    }
}

// 단순히 값을 드라이버로 전달 (UI 역할만)
void send_rotation_value(int fd, int percentage) {
    if (ioctl(fd, STEPPER_STEP, &percentage) < 0) {
        perror("회전 명령 전송 실패");
        return;
    }
    
    printf("%d%% 회전 명령을 전송했습니다.\n", percentage);
}

void stop_motor(int fd) {
    if (ioctl(fd, STEPPER_STOP) < 0) {
        perror("모터 정지 실패");
        return;
    }
    
    printf("모터 정지 명령을 전송했습니다.\n");
}

void show_help(void) {
    printf("\n=== 도움말 ===\n");

    printf("사용법 (상대적 위치 이동):\n");
    printf("0~100%% 범위의 값을 입력하세요:\n");
    printf("  입력값: 목표 위치 (창문 열림 정도)\n");
    printf("  드라이버가 현재 위치에서 목표 위치로 자동 이동\n\n");

    printf("동작 원리:\n");
    printf("- 드라이버가 현재 위치를 추적합니다\n");
    printf("- 입력값과 현재 위치의 차이만큼 이동합니다\n");
    printf("- 양수 차이: 정방향 이동\n");
    printf("- 음수 차이: 역방향 이동\n");
    printf("- 같은 값: 이동하지 않음\n\n");
}
```