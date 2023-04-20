(define (domain granny_house)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
room door - location
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_available ?r - robot)
(robot_at ?r - robot ?z - location)
(connected ?z1 ?z2)
(door_open ?d - door)
(door_closed ?d - door)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1))
        (at start(robot_available ?r))
        (at start(connected ?z1 ?z2))
        )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
)

(:durative-action move_through_door
    :parameters (?r - robot ?d - door ?z1 ?z2 - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1))
        (at start(robot_available ?r))
        (at start(connected ?z1 ?z2))
        (at start(door_open ?d))
        )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
)

(:durative-action open-door
  :parameters (?r - robot ?d - door)
  :duration (= ?duration 5)
  :condition 
      (and 
        (at start(robot_at ?d))
        (at start(door_closed ?d))
      )
  :effect 
    (and 
      (at end (door_open ?d))
      (at start(not (door_closed ?d)))
    )
)


;; end Actions ;;;;;;;;;;;;;;;;;;;;;;

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
