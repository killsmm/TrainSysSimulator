#include "Train.h"

Train::Train(TrackSegment *original_position, Terminator *destination){
    this->current = original_position;
    this->destination = destination;
    this->isMoving = false;
}

void Train::move(){
    if(this->isMoving){
        //TODO move to the next segment based on route
    }
}