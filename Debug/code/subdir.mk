################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/Attitude.c" \
"../code/MenuElement.c" \
"../code/QuaternionEKF.c" \
"../code/balance.c" \
"../code/camera.c" \
"../code/dht11.c" \
"../code/disply.c" \
"../code/element.c" \
"../code/image.c" \
"../code/image1.c" \
"../code/kalman_filter.c" \
"../code/matrix.c" \
"../code/momentumwheel.c" \
"../code/pid.c" \
"../code/servo.c" \
"../code/small_driver_uart_control.c" \
"../code/swj.c" \
"../code/vmc.c" \
"../code/zf_device_dot_matrix_screen.c" \
"../code/zf_device_lora3a22.c" \
"../code/zf_device_tld7002.c" 

COMPILED_SRCS += \
"code/Attitude.src" \
"code/MenuElement.src" \
"code/QuaternionEKF.src" \
"code/balance.src" \
"code/camera.src" \
"code/dht11.src" \
"code/disply.src" \
"code/element.src" \
"code/image.src" \
"code/image1.src" \
"code/kalman_filter.src" \
"code/matrix.src" \
"code/momentumwheel.src" \
"code/pid.src" \
"code/servo.src" \
"code/small_driver_uart_control.src" \
"code/swj.src" \
"code/vmc.src" \
"code/zf_device_dot_matrix_screen.src" \
"code/zf_device_lora3a22.src" \
"code/zf_device_tld7002.src" 

C_DEPS += \
"./code/Attitude.d" \
"./code/MenuElement.d" \
"./code/QuaternionEKF.d" \
"./code/balance.d" \
"./code/camera.d" \
"./code/dht11.d" \
"./code/disply.d" \
"./code/element.d" \
"./code/image.d" \
"./code/image1.d" \
"./code/kalman_filter.d" \
"./code/matrix.d" \
"./code/momentumwheel.d" \
"./code/pid.d" \
"./code/servo.d" \
"./code/small_driver_uart_control.d" \
"./code/swj.d" \
"./code/vmc.d" \
"./code/zf_device_dot_matrix_screen.d" \
"./code/zf_device_lora3a22.d" \
"./code/zf_device_tld7002.d" 

OBJS += \
"code/Attitude.o" \
"code/MenuElement.o" \
"code/QuaternionEKF.o" \
"code/balance.o" \
"code/camera.o" \
"code/dht11.o" \
"code/disply.o" \
"code/element.o" \
"code/image.o" \
"code/image1.o" \
"code/kalman_filter.o" \
"code/matrix.o" \
"code/momentumwheel.o" \
"code/pid.o" \
"code/servo.o" \
"code/small_driver_uart_control.o" \
"code/swj.o" \
"code/vmc.o" \
"code/zf_device_dot_matrix_screen.o" \
"code/zf_device_lora3a22.o" \
"code/zf_device_tld7002.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/Attitude.src":"../code/Attitude.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/Attitude.o":"code/Attitude.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/MenuElement.src":"../code/MenuElement.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/MenuElement.o":"code/MenuElement.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/QuaternionEKF.src":"../code/QuaternionEKF.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/QuaternionEKF.o":"code/QuaternionEKF.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/balance.src":"../code/balance.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/balance.o":"code/balance.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/camera.src":"../code/camera.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/camera.o":"code/camera.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/dht11.src":"../code/dht11.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/dht11.o":"code/dht11.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/disply.src":"../code/disply.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/disply.o":"code/disply.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/element.src":"../code/element.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/element.o":"code/element.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/image.src":"../code/image.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/image.o":"code/image.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/image1.src":"../code/image1.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/image1.o":"code/image1.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/kalman_filter.src":"../code/kalman_filter.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/kalman_filter.o":"code/kalman_filter.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/matrix.src":"../code/matrix.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/matrix.o":"code/matrix.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/momentumwheel.src":"../code/momentumwheel.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/momentumwheel.o":"code/momentumwheel.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/pid.src":"../code/pid.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/pid.o":"code/pid.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/servo.src":"../code/servo.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/servo.o":"code/servo.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/small_driver_uart_control.src":"../code/small_driver_uart_control.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/small_driver_uart_control.o":"code/small_driver_uart_control.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/swj.src":"../code/swj.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/swj.o":"code/swj.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/vmc.src":"../code/vmc.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/vmc.o":"code/vmc.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/zf_device_dot_matrix_screen.src":"../code/zf_device_dot_matrix_screen.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/zf_device_dot_matrix_screen.o":"code/zf_device_dot_matrix_screen.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/zf_device_lora3a22.src":"../code/zf_device_lora3a22.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/zf_device_lora3a22.o":"code/zf_device_lora3a22.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/zf_device_tld7002.src":"../code/zf_device_tld7002.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fH:/13333/COMPETE DOUCMENT/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/zf_device_tld7002.o":"code/zf_device_tld7002.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/Attitude.d ./code/Attitude.o ./code/Attitude.src ./code/MenuElement.d ./code/MenuElement.o ./code/MenuElement.src ./code/QuaternionEKF.d ./code/QuaternionEKF.o ./code/QuaternionEKF.src ./code/balance.d ./code/balance.o ./code/balance.src ./code/camera.d ./code/camera.o ./code/camera.src ./code/dht11.d ./code/dht11.o ./code/dht11.src ./code/disply.d ./code/disply.o ./code/disply.src ./code/element.d ./code/element.o ./code/element.src ./code/image.d ./code/image.o ./code/image.src ./code/image1.d ./code/image1.o ./code/image1.src ./code/kalman_filter.d ./code/kalman_filter.o ./code/kalman_filter.src ./code/matrix.d ./code/matrix.o ./code/matrix.src ./code/momentumwheel.d ./code/momentumwheel.o ./code/momentumwheel.src ./code/pid.d ./code/pid.o ./code/pid.src ./code/servo.d ./code/servo.o ./code/servo.src ./code/small_driver_uart_control.d ./code/small_driver_uart_control.o ./code/small_driver_uart_control.src ./code/swj.d ./code/swj.o ./code/swj.src ./code/vmc.d ./code/vmc.o ./code/vmc.src ./code/zf_device_dot_matrix_screen.d ./code/zf_device_dot_matrix_screen.o ./code/zf_device_dot_matrix_screen.src ./code/zf_device_lora3a22.d ./code/zf_device_lora3a22.o ./code/zf_device_lora3a22.src ./code/zf_device_tld7002.d ./code/zf_device_tld7002.o ./code/zf_device_tld7002.src

.PHONY: clean-code

