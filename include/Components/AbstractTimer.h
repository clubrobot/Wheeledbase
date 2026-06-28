//
// Created by awing on 6/28/26.
//
/**
 * @file AbstractTimer.h
 * @author Awing
 * @ingroup wheeledbase
 * @brief Un timer abstrait pour que Codewheel ne dépende pas directement du hardware
 */

#ifndef WHEELEDBASE_ABSTRACTTIMER_H
#define WHEELEDBASE_ABSTRACTTIMER_H

/**
 * @class AbstractTimer
 * @brief Classe abstraite pour représenter un timer
 */
class AbstractTimer {
public:
    virtual ~AbstractTimer() = default;

    /**
     * @brief Retourne la valeur actuelle du timer
     * @return Le compteur actuel du timer
     */
    [[nodiscard]] virtual unsigned long getCounter() const = 0;
};

#endif //WHEELEDBASE_ABSTRACTTIMER_H
