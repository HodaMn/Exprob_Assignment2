(define (problem exp_problem) (:domain exp_domain)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    mk1 mk2 mk3 mk4 - marker
    RosBot - robot
)

(:init
    (not_visited wp0)
    (not_visited wp1)
    (not_visited wp2)
    (not_visited wp3)
    (not_visited wp4)

    (marker_at mk1 wp0)
    (marker_at mk2 wp1)
    (marker_at mk3 wp2)
    (marker_at mk4 wp3)

    (not_detected mk1)
    (not_detected mk2)
    (not_detected mk3)
    (not_detected mk4)

    (robot_at RosBot wp4)

)

(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (detected mk1)
    (detected mk2)
    (detected mk3)
    (detected mk4)
    (all_markers_detected)
    
))
)
