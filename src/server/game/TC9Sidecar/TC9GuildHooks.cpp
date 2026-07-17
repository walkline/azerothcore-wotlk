/*
 * This file is part of the AzerothCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation; either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "TC9GuildHooks.h"
#include "Guild.h"
#include "GuildMgr.h"
#include "Item.h"
#include "ObjectAccessor.h"
#include "PetitionMgr.h"
#include "Player.h"
#include "WorldSession.h"

void ToCloud9GuildHooks::OnGuildMemberAdded(uint64 guild, uint64 character)
{
    Player *player = ObjectAccessor::FindPlayer(ObjectGuid(character));
    if (!player)
        return;

    player->SetInGuild(guild);
}

void ToCloud9GuildHooks::OnGuildMemberRemoved(uint64 /*guild*/, uint64 character)
{
    Player *player = ObjectAccessor::FindPlayer(ObjectGuid(character));
    if (!player)
        return;

    player->SetInGuild(0);
}

void ToCloud9GuildHooks::OnGuildMemberLeft(uint64 /*guild*/, uint64 character)
{
    Player *player = ObjectAccessor::FindPlayer(ObjectGuid(character));
    if (!player)
        return;

    player->SetInGuild(0);
}

// Consumes guild.created: mirrors the new guild in memory on every shard and
// cleans up the charter item and petition state. The gateway already answered
// the leader's client with the turn-in result.
void ToCloud9GuildHooks::OnGuildCreated(uint64 guildId, char* guildName, uint64 leaderGuid, uint64* memberGuids, int memberGuidsSize)
{
    ObjectGuid leader = ObjectGuid(leaderGuid);

    std::vector<ObjectGuid> members;
    if (memberGuids && memberGuidsSize > 0)
    {
        members.reserve(memberGuidsSize);
        for (int i = 0; i < memberGuidsSize; ++i)
            members.emplace_back(ObjectGuid(memberGuids[i]));
    }

    // Destroy the charter item before the petition store is purged (the item
    // lives on the shard where the leader is online).
    if (Player* player = ObjectAccessor::FindPlayer(leader))
        if (Petition const* petition = sPetitionMgr->GetPetitionByOwnerWithType(leader, GUILD_CHARTER_TYPE))
            if (Item* item = player->GetItemByGuid(petition->petitionGuid))
                player->DestroyItem(item->GetBagSlot(), item->GetSlot(), true);

    // Mirror the guild in memory unless a replayed event already did; the
    // charter/petition cleanup below must run either way so the consumer
    // stays idempotent.
    if (!sGuildMgr->GetGuildById(uint32(guildId)))
    {
        Guild* guild = new Guild();
        if (guild->MirrorClusterCreated(uint32(guildId), guildName ? guildName : "", leader, members))
            sGuildMgr->AddGuild(guild);
        else
            delete guild;
    }

    // Same cleanup the core turn-in/AddMember path performs: the leader's
    // charter rows and the members' signatures. Database deletes are
    // idempotent across shards, the local petition caches need it everywhere.
    Player::RemovePetitionsAndSigns(leader, GUILD_CHARTER_TYPE);
    for (ObjectGuid memberGuid : members)
        Player::RemovePetitionsAndSigns(memberGuid, GUILD_CHARTER_TYPE);
}
