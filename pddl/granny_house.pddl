(define (domain granny_house)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
room predoor - location
door
object
person
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_available ?r - robot)
(robot_at ?r - robot ?z - location)
(connected ?z1 ?z2 - location)
(next_to_door ?z - location ?d - door)
(door_open ?d - door)
(door_closed ?d - door)
(door_connection ?d - door ?z1 ?z2 - predoor)
(object_at ?o - object ?l - room)
(robot_carries ?r - robot ?o - object)
(arranged_object ?o - object ?l - room)
(person_at ?p - person ?l - room)

(human_request_opendoor ?d - door)
(human_request_closedoor ?d - door)
(human_request_object ?p - person ?o - object)

(no_object_request ?r - robot)
(no_open_door_request ?r - robot )
(no_close_door_request ?r - robot)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move_through_door
    :parameters (?r - robot ?z1 ?z2 - predoor ?d - door)
    :duration (= ?duration 5)
    :precondition (and
        (robot_at ?r ?z1)
        (robot_available ?r)
        (door_connection ?d ?z1 ?z2)
        (door_open ?d)
    )
    :effect (and
        (not (robot_at ?r ?z1))
        (robot_at ?r ?z2)
        (not (robot_available ?r))
        (robot_available ?r)
    )
)

(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - location)
    :duration (= ?duration 5)
    :precondition (and
        (robot_at ?r ?z1)
        (robot_available ?r)
        (connected ?z1 ?z2)
    )
    :effect (and
        (not (robot_at ?r ?z1))
        (robot_at ?r ?z2)
        (not (robot_available ?r))
        (robot_available ?r)
    )
)

(:durative-action open_door
  :parameters (?r - robot ?d - door ?z1 - predoor)
  :duration (= ?duration 5)
  :condition 
      (and 
        (at start (next_to_door ?z1 ?d))
        (at start(robot_at ?r ?z1))
        (at start(door_closed ?d))
      )
  :effect 
    (and 
      (at start(not (door_closed ?d)))
      (at end (door_open ?d))
    )
)

(:durative-action requested_door_opened
  :parameters (?d - door ?r - robot ?z1 - predoor)
  :duration (= ?duration 5)
  :condition 
      (and 
        (at start (robot_at ?r ?z1))
        (at start (next_to_door ?z1 ?d))
        (at start(human_request_opendoor ?d))
        (at end(door_open ?d))
      )
  :effect 
    (and 
      (at end (not (human_request_opendoor ?d)))
      (at end (no_open_door_request ?r))
    )
)

(:durative-action close_door
  :parameters (?r - robot ?d - door ?z1 - predoor)
  :duration (= ?duration 5)
  :condition 
      (and 
        (at start (next_to_door ?z1 ?d))
        (at start(robot_at ?r ?z1))
        (at start(door_open ?d))
      )
  :effect 
    (and 
      (at start(not (door_open ?d)))
      (at end (door_closed ?d))
    )
)

(:durative-action requested_door_closed
  :parameters (?d - door ?r - robot ?z1 - predoor)
  :duration (= ?duration 5)
  :condition 
      (and 
        (at start (robot_at ?r ?z1))
        (at start (next_to_door ?z1 ?d))
        (at start(human_request_closedoor ?d))
        (at end(door_closed ?d))
      )
  :effect 
    (and 
      (at end (not (human_request_closedoor ?d)))
      (at end (no_close_door_request ?r))
    )
)

(:durative-action pick
  :parameters (?o - object ?l - room ?r - robot)
  :duration (= ?duration 5)
  :condition 
    (and
      (at start(object_at ?o ?l))
      (at start(robot_at ?r ?l))
    )
  :effect 
    (and 
      (at end(robot_carries ?r ?o))
      (at start(not (object_at ?o ?l)))
    )
)

(:durative-action drop
  :parameters (?o - object ?l - room ?r - robot)
  :duration (= ?duration 5)
  :condition 
    (and 
      (at start(robot_at ?r ?l))
      (at start(robot_carries ?r ?o))
    )
  :effect 
    (and 
      (at end(object_at ?o ?l))
      (at start(not (robot_carries ?r ?o)))
    )
)

(:durative-action give_object
  :parameters (?o - object ?l - room ?r - robot ?p - person)
  :duration (= ?duration 5)
  :condition 
    (and 
      (at start(robot_carries ?r ?o))
      (at start(person_at ?p ?l))
      (at start(robot_at ?r ?l))
      (at start(human_request_object ?p ?o))
    )
  :effect 
    (and 
      (at end (no_object_request ?r))
      (at end (not (human_request_object ?p ?o)))
    )
)

(:durative-action arrange-object
  :parameters (?o - object ?l - room ?r - robot)
  :duration (= ?duration 50)
  :condition 
    (and 
      (at start (no_object_request ?r))
      (at start (no_close_door_request ?r))
      (at start (no_open_door_request ?r))
      (at start (object_at ?o ?l))
      (at start (robot_at ?r ?l))
    )
  :effect 
    (and 
      (at end (arranged_object ?o ?l))
    )
)

;; end Actions ;;;;;;;;;;;;;;;;;;;;;;

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
