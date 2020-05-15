# Pablo Mart?n Guti?rrez Gileta & Jos? Manuel Navarro Ram?rez
.data
 
.text
 
# Jump to main 
j main
 
#  Hanoi Tower Algorithm
#  We use the next arguments:
#  $s4 = Deep Level
#  $s1 = pointer to array origin
#  $s2 = pointer to array helper
#  $s3 = pointer to array destiny 
 
#Function to run the algorithm
hanoiTower:
    bne  $s4, 1, else 	# If Deep level is equal to 1 continue, if not go to "else"
 
    # Base case
    # Swap the values between origin and destiny (s1 & s3)
    addi $s1, $s1, -4 	
    sw   $s4, 0($s3) 
    addi $s3, $s3, 4
    sw   $zero, 0($s1)
 
    # Get the $ra value to return & increase 1 to deep level
    add  $s4, $s4, 1
    jr   $ra 
 
    # If its not the base case we continue going deeper and making changes
    else:
    	# Store return address in stack pointer
        addi $sp, $sp, -4
    	sw   $ra, 0($sp)
       
        # We switch the memory address of the towers (helper with destiny)
        # to work with them properly & reduce deep level
        add $t3, $zero, $s2
        add $s2, $zero, $s3
        add $s3, $zero, $t3
        add $s4, $s4, -1 
 
        jal hanoiTower
 
        # Swap the values between towers (Remove from s1 & add to $s2
        addi $s1, $s1, -4 
        sw   $s4, 0($s2)
        addi $s2, $s2, 4	
        sw   $zero, 0($s1)
 
        # We switch the memory address of the towers (s1 = s3, s3 = s2, s2 = s1) & Reduce deep level
        add $t0, $zero, $s1 
        add $s1, $zero, $s3			
      	add $s3, $zero, $s2
      	add $s2, $zero, $t0 
      	add $s4, $s4, -1 
 
        jal hanoiTower
       
        # We switch the memory address of the towers (s1 = s2 & s2 = s1)
        add $t5, $zero, $s1
        add $s1, $zero, $s2
        add $s2, $zero, $t5
 
 	# Recover return address & Increase deep level
        lw   $ra, 0($sp)	
        addi $sp, $sp,4			
        addi $s4, $s4, 1
       
        jr $ra 	# Return
main:
    # Assign to $s4 the size of the tower
    lui $sp, 0x1001
    addi $sp, $sp, 0xffc
    addi  $s4, $zero, 3
 
    #Pointer to the arrays (Add space to each tower acording to size)
    lui $s1, 0x1001
    sll $t7, $s4, 2
    add $s2, $t7, $s1
    add $s3, $s2, $t7
 
    add $t6, $s4, $zero # Add Size of tower to t6
 
    # We loop the towers to add the corresponding values (add values to tower one, depending on the size. Example: size: 3 adds: 3, 2, 1)
    for:
        sw   $t6, 0($s1)
        addi  $s1, $s1, 4
        addi $t6, $t6, -1
        bne  $t6, $zero, for # If t6 != 0 repeat

  # Call hanoiTower Algorithm
  jal hanoiTower
