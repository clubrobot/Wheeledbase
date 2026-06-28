//
// Created by awing on 6/28/26.
//

#ifndef WHEELEDBASE_ABSTRACTTIMER_H
#define WHEELEDBASE_ABSTRACTTIMER_H

class AbstractTimer {
public:
    virtual ~AbstractTimer() = default;
    [[nodiscard]] virtual unsigned long getCounter() const = 0;
};

#endif //WHEELEDBASE_ABSTRACTTIMER_H
