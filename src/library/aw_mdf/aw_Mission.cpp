#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>

#include <boost/format.hpp>

#include <global.h>

#include "aw_Mission.h"
#include "aw_MDFTokens.h"

#include "aw_roadNetwork.h"
#include "aw_StringTools.h"

using namespace std;

namespace vlr {

namespace rndf {

Mission::Mission(RoadNetwork* rndf) :
	NetElement(0, "Mission"), m_checkpointList(NULL), m_speedLimitList(NULL), format_version_(-1), m_rndf(rndf) {
}

Mission::~Mission() {
	clear();
}

bool Mission::loadMDF(const string & strFileName) {
	clear();

	// load file
	ifstream file(strFileName.c_str());
	string line;
	vector<string> tokens;

	if (!file.is_open() || !file.good()) {
		appendStatus((boost::format("Could not load file '%1%'") % strFileName).str());
	}
	// insert checkpoints and speed limits
	while (true) {
		tokens.clear();
		if (!getline(file, line))
			break;
		if (!line.length())
			continue;
		CStringTools::splitString(line, tokens, RNDF_DELIMITER);
		if (tokens.size()==0)
			continue;

		if (tokens[0]==MDF_MISSION_NAME)
			setName(tokens[1]);
		else if (tokens[0]==MDF_MISSION_FORMAT_VERSION)
			format_version_ = CStringTools::gfCFloat(tokens[1]);
		else if (tokens[0]==MDF_MISSION_CREATION_DATE)
			creation_date_ = tokens[1];
		else if (tokens[0]==MDF_CHECKPOINTLIST_BEGIN) {
			string strData = RoadNetwork::section(file, MDF_CHECKPOINTLIST_BEGIN, MDF_CHECKPOINTLIST_END);
			if (!strData.length())
				continue;
			addCheckpointList(strData);
		} else if (tokens[0]==MDF_MISSION_RNDF_REF) {
			m_rndfName = tokens[1];
			// TODO: (dj) shoreline_rndf_lot.txt
			//if (m_rndfName != m_rndf->name())
			//	setStatus("RNDF Name does not match: " + tokens[1]+ ", "+ m_rndf->name());
		} else if (tokens[0]==MDF_SPEEDLIMITLIST_BEGIN) {
			string strData = RoadNetwork::section(file, MDF_SPEEDLIMITLIST_BEGIN, MDF_SPEEDLIMITLIST_END);
			if (!strData.length())
				continue;
			addSpeedLimitList(strData);
		} else if (tokens[0]==MDF_MISSION_END) {
		} else
			appendStatus("Unknown token \'" + tokens[0]+ "\' in MDF data.");
	}

	file.close();

	cout << "------------------------------------" << endl;
	dump();
	cout << "------------------------------------" << endl;

	return (status().length() == 0);
}

CheckpointList * Mission::addCheckpointList(const string & strData) {
	if (m_checkpointList) {
		appendStatus("multiple checkpoint lists");
		return NULL;
	}

	std::string cpname="class rndf::CheckpointList"+name()+"CheckpointList";
	CheckpointList * pList = new CheckpointList(0, cpname);
	assert(pList);
	m_checkpointList = pList;
	istringstream iStream(strData);
	string line;
	vector<string> tokens;
	int n = -1;
	// loop lines
	while (true) {
		tokens.clear();
		if (!getline(iStream, line))
			break;
		if (!line.length())
			continue;
		CStringTools::splitString(line, tokens, RNDF_DELIMITER);
		if (tokens[0]==MDF_CHECKPOINTLIST_NUM_CHECKPOINTS)
			n = CStringTools::gnCInt(tokens[1]);
		else {
			cout << "getCheckpoint "<< tokens[0]<< " of "<< n << endl;
			CheckPoint * cp = m_rndf->checkPoint(tokens[0]);
			if (cp) {
				pList->addCheckpoint(cp);
			} else {
				cerr << "checkpoint " << tokens[0] << " could not be found -> skip it" << std::endl;
				--n;
			}
		}
	}

	if ((int32_t)pList->size() != n)
		appendStatus((boost::format("wrong number of checkpoints RNDF states %1% but has %2% defined") % n % pList->size()).str());

	return pList;
}

SpeedLimitList * Mission::addSpeedLimitList(const string & strData) {
	//cout << strData << endl;
	if (m_speedLimitList) {
		appendStatus("multiple speed limit lists");
		return NULL;
	}

	std::string sllname=name()+"SpeedLimitList";
	SpeedLimitList * pList = new SpeedLimitList(0, sllname);

	m_speedLimitList = pList;
	istringstream iStream(strData);
	string line;
	vector<string> tokens;
	int n = -1;
	// loop lines
	while (true) {
		tokens.clear();
		if (!getline(iStream, line)) {
			break;
		}
		if (!line.length())
			continue;
		CStringTools::splitString(line, tokens, RNDF_DELIMITER);
		if (tokens[0]==MDF_SPEEDLIMITLIST_NUM_SPEEDLIMITS) {
			n = CStringTools::gnCInt(tokens[1]);
		} else if (tokens.size()> 2) {
			SpeedLimit * limit = addSpeedLimit(tokens[0], tokens[1], tokens[2]);
			pList->addSpeedLimit(limit);
		} else {
			cout << line << " could not be parsed" << endl;
		}
	}

	if (pList->size() != n)
		appendStatus("wrong number of speed limits");

	return pList;
}

SpeedLimit * Mission::addSpeedLimit(const string & strName, const string & strMin, const string & strMax) {
	std::string sllname=strName;
	SpeedLimit* pSpeedLimit = new SpeedLimit(0, sllname);
	assert(pSpeedLimit);

	cout << "speed limit for " << strName << endl;
	Segment* seg = m_rndf->getSegment(strName);
	if (pSpeedLimit == NULL) {
		appendStatus("addSpeedLimit: speed limit was not added to the network.");
		return NULL;
	}
	if (seg == NULL) {
		Zone* myzone = m_rndf->zone(strName);
		if (myzone == NULL) {
		  std::cout << "WARNING: speed limit list: segment/zone not found: " << strName << "\n";
		  //			appendStatus("speed limit list: segment/zone not found "+strName);
			return pSpeedLimit;
		}
		pSpeedLimit->setZone(myzone);
	} else {
		pSpeedLimit->setSegment(seg);
		//cout << " seg found and set" << endl;
	}
	pSpeedLimit->minSpeed(dgc_mph2ms(CStringTools::gdCDouble(strMin)));
	pSpeedLimit->maxSpeed(dgc_mph2ms(CStringTools::gdCDouble(strMax)));
	pSpeedLimit->setRefName(strName);
	return pSpeedLimit;
}

void Mission::dump() {
	cout << "Dumping mission "<< name() << "..."<< endl;
	cout << "Creation Date: "<< creation_date_ << endl;
	cout << "Format Version: "<< format_version_ << endl;
	cout << "RNDF: "<< m_rndfName << endl;
	if (m_checkpointList)
		m_checkpointList->dump();
	if (m_speedLimitList)
		m_speedLimitList->dump();
}

void Mission::clear() {
	cout << "clear mission"<< endl;
	if (m_checkpointList) {
		delete m_checkpointList;
		m_checkpointList = NULL;
	}
	if (m_speedLimitList) {
		for (TSpeedLimitIterator it = m_speedLimitList->begin(); it != m_speedLimitList->end(); ++it) {
			SpeedLimit * limit = *it;
			delete limit;
		}
		delete m_speedLimitList;
	}
	m_speedLimitList = NULL;
	setStatus("");
}

void Mission::changeSpeedLimit(SpeedLimit * limit, const string & strMin, const string & strMax) {
	double min = CStringTools::gdCDouble(strMin);
	double max = CStringTools::gdCDouble(strMax);
	if (min > limit->minSpeed()) {
		limit->minSpeed(min);
	}
	if (max < limit->maxSpeed()) {
		limit->maxSpeed(max);
	}
}


}

} // namespace vlr
