"""
Kinematic-chain management.

``ChainManager`` keeps track of the ordered lists of module nodes that form
the robot's kinematic chains (``listofchains``) and the list of hub modules
(``listofhubs``).  It also provides the static query helpers used both
internally and by the plugin back-ends.
"""
from __future__ import print_function

from modular.enums import ModuleClass, ModuleType


class ChainManager:
    """Manages the kinematic chains and hub list of the robot model.

    Parameters
    ----------
    writer : UrdfWriter
        Back-reference to the owning writer.  Used to access
        ``writer.listofchains``, ``writer.listofhubs``, and
        ``writer.inverse_branch_switcher``.
    """

    def __init__(self, writer):
        self._writer = writer

    # ------------------------------------------------------------------
    # Chain mutation
    # ------------------------------------------------------------------

    def add_to_chain(self, new_joint):
        """Append *new_joint* to the appropriate kinematic chain.

        A new chain list is created when *new_joint* belongs to a branch that
        has not been started yet (tag index > parent tag index).  Otherwise the
        module is appended to the existing chain identified by the tag index.

        Parameters
        ----------
        new_joint : ModuleNode.ModuleNode
        """
        tag_index = self._writer.inverse_branch_switcher.get(new_joint.tag)
        parent_tag_index = self._writer.inverse_branch_switcher.get(new_joint.parent.tag)
        chain = [new_joint]
        self._writer.print("tag_index: ", tag_index,
                           "list of chains: ", len(self._writer.listofchains))
        if tag_index > parent_tag_index:
            self._writer.listofchains.append(chain)
        else:
            self._writer.listofchains[tag_index].append(new_joint)

    def remove_from_chain(self, joint):
        """Remove *joint* from whatever chain it currently belongs to.

        Empty chains are pruned afterwards.

        Parameters
        ----------
        joint : ModuleNode.ModuleNode
        """
        for chain in self._writer.listofchains:
            if joint in chain:
                chain.remove(joint)
        self._writer.listofchains = list(filter(None, self._writer.listofchains))

    # ------------------------------------------------------------------
    # Chain queries
    # ------------------------------------------------------------------

    def get_actuated_modules_chains(self):
        """Return only the chains that contain at least one actuated module."""
        active_modules_chains = []
        for modules_chain in self._writer.listofchains:
            joint_num = sum(
                1 for m in modules_chain
                if m.type in ModuleClass.actuated_modules()
            )
            if joint_num > 0:
                active_modules_chains.append(modules_chain)
        return active_modules_chains

    # ------------------------------------------------------------------
    # Static helpers (no writer state needed)
    # ------------------------------------------------------------------

    @staticmethod
    def find_chain_tip_link(chain):
        """Return the name of the tip link for the given chain."""
        if chain[-1].type in ModuleClass.joint_modules():
            return chain[-1].distal_link_name
        elif chain[-1].type in ModuleClass.link_modules() | ModuleClass.hub_modules():
            return chain[-1].name
        elif chain[-1].type in ModuleClass.end_effector_modules() - {ModuleType.DAGANA}:
            return chain[-1].tcp_name
        elif chain[-1].type is ModuleType.DAGANA:
            return chain[-1].base_link_name

    @staticmethod
    def find_chain_base_link(chain):
        """Return the name of the base link for the given chain."""
        if not chain[0].parent:
            return chain[0].name
        if "con_" in chain[0].parent.name:
            return chain[0].parent.parent.name
        else:
            if not chain[0].parent.is_structural and chain[0].parent.type in ModuleClass.hub_modules():
                return chain[0].parent.parent.name
            else:
                return chain[0].parent.name

    @staticmethod
    def find_chain_tag(chain):
        """Return the branch tag letter of the last module in the chain."""
        return chain[-1].tag
