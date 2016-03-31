#include "Arduino.h" //Necessary to include Serial in this scope.

#ifndef lol_its_a_header //The thing being defined must not show up in the code
#define lol_its_a_header



//Print function definition
void myPrintFunction() {
	Serial.println("This is a test");	
}

#endif //Ends the header file definition
