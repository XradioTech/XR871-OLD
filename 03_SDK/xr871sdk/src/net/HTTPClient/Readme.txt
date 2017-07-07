
 HTTPClient 1.1 10 Nov 2014

 Author: Eitan Michaelson: noyasoft@gmail.com
 Linux Porting and testing: Bob Wirka: bobwirka@rtcworks.com
 
 DESCRIPTION
 -----------

 Highly portable API written in C that implements the client side of the HTTP 1.1 
 protocol as defined in RFC 2616,2617. 
 Can be ported to any platform that supports standard C and Berkeley sockets.           

 OVERVIEW
 --------

 The HTTPClient API includes:

 'API' Subdirectory:
     Implementation of HTTP 1.1 Protocol along with the Digest and Basic authentication shams

 HTTPClientSample.c:
     a simple demonstration of the API capabilities. This is a command line utility 
     project that use the API to do HTTP Get and retrieve a requested url.

  MSVC project files


 SUPPORT
 -------

 If you have any problems with HTTPClient then please take the following steps
 first:

    - Download the current snapshot from SourceForge
      to see if the problem has already been addressed
    - See if this problem is not a known issue with the current release.
    - Remove compiler optimization flags

 If you wish to report a bug then please include the following information in
 any bug report:

    - HTTPClient version.
    - OS Name, Version, Hardware platform
    - Compiler Details (name, version)
    - Application Details (name, version)
    - Problem Description (steps that will reproduce the problem, if known)
    - Stack Traceback (if the application dumps core)

 Report the bug to the HTTPClient author e-mail:

    noyasoft@netvision.net.il

 LICENSE
 -------
 HTTPClient is distributed under the MIT License:

 Copyright (c) 2006 Eitan Michaelson

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 and associated documentation files (the "Software"), to deal in the Software without restriction, 
 including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished 
 to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies 
 or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
 OTHER DEALINGS IN THE SOFTWARE.
