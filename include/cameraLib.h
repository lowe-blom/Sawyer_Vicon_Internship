#include <VimbaCPP/Include/VimbaCPP.h> //Vimba API
#include <include.h>

using namespace AVT::VmbAPI;

namespace my_cam
{   
    class myCamClass
    {
        public: myCamClass() : system(VimbaSystem::GetInstance())
            {      
                std::string name;
                InterfacePtrVector interfaces;
                
                if ( VmbErrorSuccess == system.Startup() )
                {
                    if ( VmbErrorSuccess == system.GetInterfaces(interfaces))
                    {
                        for ( InterfacePtrVector::iterator iter = interfaces.begin(); interfaces.end() != iter; ++ iter )
                        {
                            if ( VmbErrorSuccess == (*iter)->GetName(name) )
                            {
                                std::cout << "camera " << *iter << ": " << name << std::endl;
                            }
                        }
                    }
                }
            }

            VimbaSystem& getSystem()
            {
                return system;
            }

            /** Print list of camera's
            *
            * @brief Print full list of camera's connected on the network
            * 
            * @param system VimbaSystem object
            * 
            * @return void
            */
            void printCamList(VimbaSystem& system);

            /** Shutdown Vimba API
            *
            * @brief shutdown Vimba API
            * 
            * @param system VimbaSystem object
            * 
            * @return void
            */
            void shutdownVimba(VimbaSystem& system);

        private:
            VimbaSystem& system;
    };
}