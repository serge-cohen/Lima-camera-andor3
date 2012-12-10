/*
 *  andor3.cp
 *  andor3
 *
 *  Created by Serge Cohen on 28/03/12.
 *  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include <iostream>
#include "andor3.h"
#include "andor3Priv.h"

void andor3::HelloWorld(const char * s)
{
	 andor3Priv *theObj = new andor3Priv;
	 theObj->HelloWorldPriv(s);
	 delete theObj;
};

void andor3Priv::HelloWorldPriv(const char * s) 
{
	std::cout << s << std::endl;
};

