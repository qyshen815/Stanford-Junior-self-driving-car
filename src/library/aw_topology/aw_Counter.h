/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_COUNTER_H
#define AW_COUNTER_H

namespace vlr {

namespace rndf {

struct Counter {
    public:
      Counter() { count = 0; }
      int getNextId() { return ++count; }
    private:
      int count;
  };

}

} // namespace vlr

#endif

/***************************************************************************
 *
 * $Log: Counter.h,v $
 * Revision 1.1  2007/05/08 14:35:41  frese
 * cleaned up
 *
 *
 */
