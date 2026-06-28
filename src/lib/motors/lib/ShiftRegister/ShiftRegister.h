/**
 * @file ShiftRegister.h
 * @ingroup motors
 * @brief Gestion d'un registre à décalage.
 */

#ifndef __SHIFTREGISTER_H__
#define __SHIFTREGISTER_H__

#include <Arduino.h>

class ShiftRegister
{
	public:
		/**
		 * @brief Configure les broches du registre.
		 * @param latchpin Broche latch.
		 * @param clockpin Broche clock.
		 * @param datapin Broche data.
		 */
		void attach(uint8_t latchpin, uint8_t clockpin, uint8_t datapin);
		/**
		 * @brief Positionne une sortie à l'état haut.
		 * @param pos Numéro de la sortie.
		 */
		void SetHigh(int pos);
		/**
		 * @brief Positionne une sortie à l'état bas.
		 * @param pos Numéro de la sortie.
		 */
		void SetLow (int pos);
		/**
		 * @brief Écrit un état sur une sortie.
		 * @param pos Numéro de la sortie.
		 * @param state État à appliquer.
		 */
		void write(int pos, int state);

	protected:
		/**
		 * @brief Décale le contenu du registre vers les sorties.
		 */
		void shift();

		uint8_t m_LATCH;
		uint8_t m_CLOCK;
		uint8_t m_DATA;

		volatile uint8_t m_register;
};

//extern ShiftRegister shift;
#endif // __SHIFTREGISTER_H__