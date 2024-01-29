.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 5T(n/2) + 6n + 4, T(1) = 2\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text


__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall

  # Reads an int
    addi a0, x0, 5
    ecall
    

################################################################################ 
  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 5T(n/2) + 6n + 4, T(1) = 2, round down the result of division
  # ex. addi t0, a0, 1
  
  # Check if n is 1, if so, set t0 to 2 and return
  
    jal x1, calculate_T    # begin calculate T(n)
    mv t0, a0              # after ending calculate T(n), mv the result to t0
    j result               # jump to result to print answer
  
calculate_T:
    addi sp, sp, -8        # use stack pointer to store loacl variable and return address
    sw x1, 4(sp)           # sotre lcoal variable and return address into memory
    sw a0, 0(sp)           
    addi x7, a0, -1        # in order to compare thea0 and 1
    bgt x7, x0, recursive  # if a0 > 1, jump to recursive
    addi a0 x0, 2          # if a0 == 1, set a0 to 2
    addi sp, sp, 8         # pop the memory stack, because a0 == 1 is the base case, don't need to load back the value of x1 and a0 (no change)
    jalr x0 0(x1)          # use x1 return to recursive (mv x8, a0) 
    
    
recursive:
    srli a0, a0, 1         # need n/2 -> right shift to achieve divide 2 and round down (floor)
    jal x1, calculate_T    # calculate T(n/2), store return address (return back to recursive mv x8, a0) to x1
    mv x8, a0              # use temp reg x8 to store the result of T(n/2) 
    lw a0, 0(sp)           # load back local variable a0 and return address x1 
    lw x1, 4(sp)
    addi sp, sp, 8         # after load, pop the stack
    li x9, 6               # store constant multiplier 6
    mul a0, a0, x9         # calculate 6n 
    li x9, 5               # store constant multiplier 5
    mul x8, x8, x9         # calculate 5T(n/2)
    add a0, a0, x8         # add 5T(n/2) + 6n
    addi a0, a0, 4         # add 5T(n/2) + 6n + 4
    jalr x0, 0(x1)         # return back to recursive mv x8, a0

################################################################################

result:
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall        