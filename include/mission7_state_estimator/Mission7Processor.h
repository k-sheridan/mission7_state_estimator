#include "Types.h"

/*
* Contains the process function for a general state and uncertainty propagation via the Unscented Transform
*/
class Mission7Processor{

  Mission7State process(Mission7State state, double dt);

  Mission7State processAndPropagateUncertainty(Mission7State state, double dt);

};

