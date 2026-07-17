#include "petition-api.h"

// CanTurnInGuildPetition
static CanTurnInGuildPetitionHandler canTurnInGuildPetitionHandler;
void SetCanTurnInGuildPetitionHandler(CanTurnInGuildPetitionHandler h) {
    canTurnInGuildPetitionHandler = h;
}

GuildPetitionValidationResult CallCanTurnInGuildPetitionHandler(uint64_t player_guid, uint64_t petition_item_guid) {
    if (canTurnInGuildPetitionHandler == 0) {
        GuildPetitionValidationResult result;
        result.status = GuildPetitionCheckStatusNoHandler;
        result.guildName = 0;
        result.signatoryGUIDs = 0;
        result.signatoryGUIDsSize = 0;
        return result;
    }
    return canTurnInGuildPetitionHandler(player_guid, petition_item_guid);
}
