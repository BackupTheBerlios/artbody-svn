// ChainApp.cpp : Defines the entry point for the application.
//
#include <windows.h>
#include "ChainApp.h"

app::Application app::App;

using namespace app;

Application::Application()
{
}

Application::~Application()
{
}

bool Application::Init(int argc, char * argv[])
{
   bool rc = true;

   if (rc) {
      rc &= _InitSystem(argc, argv);
   }

   return rc;
}

void Application::Term(void)
{
   _TermSystem();
   
   return;
}

int main(int argc, char * argv[])
{
   int rc = 0;

   if ( App.Init(argc, argv) ) {
      rc = App.MainLoop();
   }


   App.Term();

	return rc;
}

