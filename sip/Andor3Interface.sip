namespace Andor3
{
  class Interface : HwInterface
  {
%TypeHeaderCode
#include <Andor3Interface.h>
%End
  public:
    Interface(Andor3::Camera& cam);
    virtual ~Interface();

    //- From HwInterface
    //    virtual void 	getCapList(CapList& /Out/) const;
    virtual void	getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void	reset(ResetLevel reset_level);
    virtual void 	prepareAcq();
    virtual void 	startAcq();
    virtual void 	stopAcq();
    virtual void 	getStatus(StatusType& status /Out/);
    virtual int 	getNbHwAcquiredFrames();

  };
};
