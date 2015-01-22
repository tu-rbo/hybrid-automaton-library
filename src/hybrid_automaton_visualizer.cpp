#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/JointConfigurationSensor.h"
#include "hybrid_automaton/ForceTorqueSensor.h"
#include "hybrid_automaton/ClockSensor.h"
#include "hybrid_automaton/DescriptionTreeXML.h"

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>

using namespace ha;

std::string loadXMLFile(const char* filename) {

    std::ifstream t(filename);
    if (!t.good()) {
        t.close();
        HA_THROW_ERROR("hybrid_automaton_visualizer", "File not found: " << filename);
    }
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());

    t.close();
    return str;
}

namespace ha {

    class DummySystem : public System {

    protected:

    public:
        DummySystem() {
        }

        virtual ~DummySystem() {
        }

        virtual int getDof() const { return 0; }
        virtual ::Eigen::MatrixXd getConfiguration() const  { ::Eigen::MatrixXd m; return m; }
        virtual ::Eigen::MatrixXd getForceTorqueMeasurement() const { ::Eigen::MatrixXd m; return m; }
        virtual ::Eigen::MatrixXd getFramePose(const std::string&) const { ::Eigen::MatrixXd m; return m; }

    };
}

int main(int argc, char *argv[]) {

    if (argc < 2) {
        HA_ERROR("hybrid_automaton_visualizer", "Usage: ./hybrid_automaton_visualizer <xml file> [<out file (without extension)>]");
    }

    // generate file names for output
    std::string out_file;
    std::string pdf_file;
    if (argc == 3) {
        out_file = argv[2];
        pdf_file = argv[2];
    } else {
        std::string raw(argv[1]);
        std::string ext;
        if (raw.length() >= 4)
            ext = raw.substr( raw.size()-3,  raw.size());
        if (ext == "xml")
            raw = raw.substr(0, raw.size()-4); // remove .xml
        out_file = raw;
        pdf_file = raw;
    }
    out_file += ".dot";
    pdf_file += ".pdf";

    // read XML string

    HybridAutomaton ha;
    ha.setDeserializeDefaultEntities(true);
    //System::Ptr sys(new DummySystem);
    System::Ptr sys; // NULL on purpose

    std::cout << "Reading " << argv[1] << std::endl;
    DescriptionTreeXML dt(loadXMLFile(argv[1]));
    ha.deserialize(dt.getRootNode(), sys);

    ha.visualizeGraph(out_file);

    std::stringstream cmd;
    cmd << "dot -Tpdf " << out_file << " -o " << pdf_file;
    std::cout << "Executing " << cmd.str() << std::endl;
    system(cmd.str().c_str());

    std::cout << "Successfully created " << std::endl;
    std::cout << " " << out_file << std::endl;
    std::cout << " " << pdf_file << std::endl;


}
