# Notice: Clearpath Robotics `clearpath_simulator` (BSD-3-Clause)

`warehouse_world.sdf` in this directory is adapted from
`clearpath_gz/worlds/warehouse.sdf` in <https://github.com/clearpathrobotics/clearpath_simulator> (branch `jazzy`).

**Inherited:** the scene layout, i.e. the list of Gazebo Fuel model includes with their names and poses (25 of 25 identical to upstream
at the time of comparison, 2026-11-03).
**Changed in this project:** world name, SDF version, plugin spelling, physics engine and step, scene and lighting, ground plane,
removal of NavSat and spherical coordinates, and the added `apriltag_station_a` include.

This project is independent and is not affiliated with or endorsed by Clearpath Robotics. The BSD-3-Clause licence below covers
only the Clearpath-authored material. It does not cover the Gazebo Fuel models the world references; see
[`THIRD_PARTY_NOTICES.md`](../../../THIRD_PARTY_NOTICES.md) and [`THIRD_PARTY_REVIEW.md`](../../../THIRD_PARTY_REVIEW.md).

## Upstream licence text (verbatim)

    BSD 3-Clause License
    
    Copyright (c) 2023, clearpathrobotics
    
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    
    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    
    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
