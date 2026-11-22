// orb_vocabulary:// : loads an ORB vocabulary (map::Vocabulary's binary file, tools/orb_vocabulary) once at
// configure and sends it on `vocabulary`. orb_vocabulary://ORBvoc.bin is <project>/orb_vocabulary/ORBvoc.bin.
//
// Read only and shared by tracking and local mapping (later loop closing): a plugin rather than a resource with
// store:, which would also save it back at the end of a run. In one box the readers share this one copy.
#include <string>

#include "fw/fw.hpp"
#include "types/map/vocabulary.hpp"

namespace {

struct OrbVocabularyPlugin {
    void on_configure(fw::PluginContext& ctx) {
        map::Vocabulary v;
        v.loadBinary(ctx.path());  // throws when missing or malformed: the box does not start
        ctx.report(fw::Health::ok, std::to_string(v.words()) + " words, k " + std::to_string(v.branching()) +
                                       ", L " + std::to_string(v.levels()));
        ctx.emit("vocabulary", std::move(v));
    }
};

}  // namespace

FW_REGISTER_FS_PLUGIN(OrbVocabularyPlugin, PREFIX("orb_vocabulary"),
    OUTPUTS(("vocabulary", map::Vocabulary, GUARANTEED)),
    ON_CONFIGURE(&OrbVocabularyPlugin::on_configure),
    ONE_INSTANCE);
