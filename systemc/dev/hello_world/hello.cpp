 // All systemc modules should include systemc.h header file
#include <systemc.h>

// Hello_world is module name
SC_MODULE (hello_world) {

   // constructor
   SC_CTOR (hello_world) {
      // Nothing in constructor
   }

   void say_hello() {
      //Print "Hello World" to the console.
      cout << "Hello World.\n";
   }
}; // end module

// sc_main in top level function like in C++ main
int sc_main(int argc, char* argv[]) {
   hello_world hello("HELLO");
   // Print the hello world
   hello.say_hello();
   return(0);
} // end sc_main
