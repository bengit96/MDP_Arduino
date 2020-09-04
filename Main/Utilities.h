//Utilities


void bubbleSort(int numIteration, unsigned long * timeWidth) {
  int out, in, swapper;
  for(out=0 ; out < numIteration; out++) {  // outer loop
    for(in=out; in<(numIteration-1); in++)  {  // inner loop
      if( timeWidth[in] > timeWidth[in+1] ) {   // out of order?
        // swap them:
        swapper = timeWidth[in];
        timeWidth [in] = timeWidth[in+1];
        timeWidth[in+1] = swapper;
      }
    }
  }
}
