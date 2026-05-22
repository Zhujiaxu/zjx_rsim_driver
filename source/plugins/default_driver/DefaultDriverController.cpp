/*
 * DefaultDriverController -- minimal plugin controller.
 *
 * Step 1 (恒速): Sets each controlled actor's longitudinal speed to setSpeed
 * every tick. Lateral / pose are left to scene_runner (only speed is reported).
 *
 * Properties (xosc):
 *   setSpeed -- target longitudinal speed in m/s, default 10.0
 */

#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <cstdio>
#include <cstring>

using namespace rsim_plugin;

namespace
{

class DefaultDriverController : public IPluginController
{
public:
    void Init(const std::map<std::string, std::string>& properties,
              const std::vector<int32_t>&                controlled_actor_ids) override
    {
        controlled_ids_ = controlled_actor_ids;

        auto it = properties.find("setSpeed");
        if (it != properties.end())
        {
            try
            {
                set_speed_ = std::stod(it->second);
            }
            catch (...)
            {
                std::fprintf(stderr,
                             "[DefaultDriver] invalid setSpeed='%s', falling back to %.2f\n",
                             it->second.c_str(),
                             set_speed_);
            }
        }

        std::fprintf(stderr,
                     "[DefaultDriver] Init: setSpeed=%.2f m/s, controlled_ids=%zu\n",
                     set_speed_,
                     controlled_ids_.size());
    }

    std::vector<ActorUpdate> Step(const TickContext& ctx) override
    {
        (void) ctx;
        std::vector<ActorUpdate> updates;
        updates.reserve(controlled_ids_.size());

        for (int32_t id : controlled_ids_)
        {
            ActorUpdate u {};
            u.actor_id       = id;
            u.speed          = set_speed_;
            u.speed_valid    = 1;
            u.position_valid = 0;
            updates.push_back(u);
        }

        return updates;
    }

private:
    std::vector<int32_t> controlled_ids_ {};
    double               set_speed_ = 10.0;
};

}  // namespace

extern "C" IPluginController* CreateController(const char* name)
{
    (void) name;
    return new DefaultDriverController();
}

extern "C" void DestroyController(IPluginController* p)
{
    delete p;
}
