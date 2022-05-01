#include "../state/config.h"
#include "../state/state.h"

class Subsystem {
  public:
    void setup(struct config, struct *state);
    void loop(float deltaTime);

  private:
    struct *state;
};