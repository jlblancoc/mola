/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MinimalModuleContainer.cpp
 * @brief  A simple module container for use without mola_launcher
 * @author Jose Luis Blanco Claraco
 * @date   Dec 26, 2024
 */

#include <mola_kernel/MinimalModuleContainer.h>

namespace mola
{
MinimalModuleContainer::~MinimalModuleContainer() = default;

void MinimalModuleContainer::installNameServer(ExecutableBase& m)
{
    m.nameServer_ = std::bind(
        &MinimalModuleContainer::nameServerImpl, this, std::placeholders::_1);
}

ExecutableBase::Ptr MinimalModuleContainer::nameServerImpl(
    const std::string& name)
{
    // Special syntax to sequentially access all existing modules:
    // If the requested name has the format: "[" + <i>, return the i-th
    // module, or nullptr if out of range.
    // This is used by ExecutableBase::findService()
    if (name.size() >= 2 && name[0] == '[')
    {
        const auto idx = std::stoul(name.substr(1));
        if (idx >= modules_.size()) { return ExecutableBase::Ptr(); }
        else
        {
            auto it = modules_.begin();
            std::advance(it, idx);
            return *it;
        }
    }
    // non numeric search not implemented in this minimal container
    return ExecutableBase::Ptr();
}

}  // namespace mola
