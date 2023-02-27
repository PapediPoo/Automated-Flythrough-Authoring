# Automated Flythrough Authoring

**Note:** This project served as my B.Sc thesis at ETH Zurich from 2021.

The visualization and presentation of architectural models in virtual reality enables architects to evaluate their designs and present their architectural models to stakeholders,
improving the fidelity of architectural communication in architectural design. Stakeholders can better examine the models in an immersive environment and have a better
understanding of the final product, which helps to align stakeholder expectations with
the actual design.

Virtual walkthrough or flythrough animations, also known as trajectories, are a powerful tool to convey the architectural concept and the unique features of the planned building.
However, creating such animations is a rather tedious process.
It involves manually setting keypoints at the places of interest and in between, which are then interpolated to create the trajectory.
This often results in awkward camera poses, sharp turn, intersection with geometry and clipping problems, etc., all of which must be resolved by tweaking or adding new keypoints.
Furthermore, the process requires the designer to constantly switch between an inside view of the structure, for local refinements, and an outside overview, for more global adjustments.

We address these issues using our novel, interactive design tool for walkthroughs in VR.
Our tool automatically provides a set of ideal keypoints, based on a customizable set of objectives, such as spatial coverage and visibility.
Utilizing trajectory optimization and path planning techniques, our tool instantly generates a complete and smooth trajectory, that passes through all keypoints and does not clip through geometry.
The efficiency of the algorithms allows our tool to run at interactive rates, in the sense that the designer is able to tweak the positions of the keypoints, or any other parameters, and observe the newly computed trajectory instantaneously.
In addition, our VR interface allows the designer to have both an inside view and and overview simultaneously, which leads to improved productivity.

We have published a paper about this topic, which can be viewed [HERE](https://caadria2022.org/automatic_flythrough_authoring_in_vr/).
