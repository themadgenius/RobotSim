

// subscribe to thing that listens for physics done message
// subscribe to thing tht listens for embedded done message

while(1){
// Take 1 semaphore for physics
// publish message to start physics engine
// Take 1 semaphore for embedded
// publish message to start embedded engine
// if both semaphores are back then continue loop
// while (both semaphores aren't back){
//       ros::spinOnce(); // check for messages
// } // end while semaphores
} // end main loop



