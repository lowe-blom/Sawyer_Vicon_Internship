#include <cameraLib.h>
using namespace AVT::VmbAPI;

namespace my_cam
{
    void myCamClass::printCamList(VimbaSystem& system)
    {
        std::string name;
        InterfacePtrVector interfaces;
        
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

    void myCamClass::shutdownVimba(VimbaSystem& system)
    {
        system.Shutdown();
    }




}