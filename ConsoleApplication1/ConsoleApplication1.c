#include <stdio.h>

int main() {
    int num, is_prime, count;


    printf("Enter an integer: ");
    scanf_s("%d", &num);

  
    if (num <= 1) {
        printf("%d is not a prime number.\n", num);
        return 1;
    }

    is_prime = 1;
    count = 0;

  
    for (int i = 2; i * i <= num; i++) {
        count++;
        if (num % i == 0) {
            is_prime = 0;
            break;
        }
    }

    
    if (is_prime) {
        printf("%d is a prime number (iterations: %d).\n", num, count);
    }
    else {
        printf("%d is not a prime number (iterations: %d).\n", num, count);
    }

    return 0;
}

