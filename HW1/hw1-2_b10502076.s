.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Enter shift: "
    msg2: .string "Plaintext: "
    msg3: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg3
    ecall
  
    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
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
    add a6, a0, x0
    
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    addi a0,x0,8
    li a1, 0x10150
    addi a2,x0,2047
    ecall
  # Load address of the input string into a0
    add a0,x0,a1


################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # x16 stores the shift
  # Do store 66048(0x10200) into x20 
  # ex. j print_char
    li x11, 97              # the ASCII of a
    li x12, 122             # the ASCII of z
    li x13, 32              # the ASCII of space
    li x14, 47              # (the ASCII of 0) - 1 
    li x15, 10              # the ASCII of "\n"
    mv x18, x0              # counter for number to replace space
    li x20, 66048           # store the output memory address to x20
    
read_input:                 # load input char and compare some cases
    lb x21, 0(a0)           # load one byte (char) of input from memory  
    beq x21, x15, Exit      # if char == "\n" -> end
    beq x21, x13, space     # if char == "space" -> jump to space
    add x21, x21, a6        # shift char 
    blt x21, x11, under_a   # after shifting , if char < a -> jump under_a
    bgt x21, x12, over_z    # after shifting , if char > z -> jump over_z
    
write_output:               # write the shift char to output memory
    sb x21, 0(x20)          # store shift char
    addi x20, x20, 1        # because char is one byte, input address (a0) and output address (x20) just need to add 1 to point to the next char
    addi a0, a0, 1          
    jal x1, read_input      # after write output back to read input 
  
over_z:
    addi x21, x21, -26      # over z -> circle from a to start
    j write_output          
    
under_a:
    addi x21, x21, 26       # under a -> circle from z to start
    j write_output
    
space:
    addi x18, x18, 1        # the counter for number to replace space + 1
    add x21, x14, x18       # the ASCII of the number to replace space
    j write_output

Exit:
    li x20, 66048           # x20 point to the first output char
    j print_char            
################################################################################

