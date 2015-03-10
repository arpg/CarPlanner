/*
 * MochaException.h
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#ifndef MOCHAEXCEPTION_H_
#define MOCHAEXCEPTION_H_

#include <iostream>
#include <exception>
using namespace std;



class MochaException : public exception
{
private:
	const char *m_pWhat;
public:
	MochaException(const char *what);
	virtual ~MochaException()  throw() ;
	virtual const char* what();
};


#endif /* MOCHAEXCEPTION_H_ */
