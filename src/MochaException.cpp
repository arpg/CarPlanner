/*
 * MochaException.cpp
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#include "CarPlanner/MochaException.h"



MochaException::MochaException(const char *what): m_pWhat(what) {
}

MochaException::~MochaException() throw()
{

}

const char* MochaException::what()
{
	return m_pWhat;
}


