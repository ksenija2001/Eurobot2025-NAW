#include "utils.h"

void bytes_to_float(uint8_t* start, float* dest){
    memcpy(start, dest, sizeof(float));
}

void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 
 
// Converts a given integer x to string str[]. 
// d is the number of digits required in the output. 
// If d is more than the number of digits in x, 
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
 
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
 
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
 
// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
 
    // Extract floating part 
    float fpart = n - (float)ipart; 
 
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
 
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
 
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter 
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
 
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 

void float_to_string(float num, char* dest){
    ftoa(num, dest, 3);
}

uint8_t factorial(uint8_t num)
{
    if (num == 0 || num == 1) return 1;
    return num * factorial(num - 1);
}

float distance_calc(float x1, float y1, float x2, float y2)
{
    return sqrtf((poow(x2-x1, 2)) + (poow(y2-y1, 2)));
}

float magnitude(float s1, float s2)
{
    return sqrtf(poow(s1, 2) + poow(s2, 2));
}

float poow(float a, uint8_t b){
    float c = a;
    if (b == 0) c = 1;
    else if (b == 1) c = a;
    else {
        for(int i = 1;i<b;i++){
            c *= a;
        }
    }
    return c;
}
