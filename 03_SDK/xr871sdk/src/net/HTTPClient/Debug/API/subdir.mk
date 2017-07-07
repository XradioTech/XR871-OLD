################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../API/HTTPClient.c \
../API/HTTPClientAuth.c \
../API/HTTPClientString.c \
../API/HTTPClientWrapper.c 

OBJS += \
./API/HTTPClient.o \
./API/HTTPClientAuth.o \
./API/HTTPClientString.o \
./API/HTTPClientWrapper.o 

C_DEPS += \
./API/HTTPClient.d \
./API/HTTPClientAuth.d \
./API/HTTPClientString.d \
./API/HTTPClientWrapper.d 


# Each subdirectory must supply rules for building sources it contributes
API/%.o: ../API/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cygwin C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


