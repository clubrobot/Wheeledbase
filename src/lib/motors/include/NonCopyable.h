/**
 * @file NonCopyable.h
 * @ingroup motors
 * @brief Classe de base pour interdire la copie.
 */

#ifndef __NONCOPYABLE_H__
#define __NONCOPYABLE_H__

/**
 * @brief Classe à hériter pour empêcher la copie.
 */
class NonCopyable
{
protected:

	NonCopyable() {}

private:
	/**
	 * @brief Constructeur par recopie déclaré privé.
	 * @param other Objet à copier.
	 */
	NonCopyable(const NonCopyable& other);

	/**
	 * @brief Opérateur d'affectation déclaré privé.
	 * @param other Objet à comparer.
	 */
	NonCopyable& operator=(const NonCopyable& other);
};

#endif // __NONCOPYABLE_H__
