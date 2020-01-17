"""Implementation of LinkedList class."""

from __future__ import annotations

from collections.abc import MutableSequence
from dataclasses import dataclass
from typing import Generic, Iterable, Tuple, TypeVar

from dataslots import with_slots

LINKED_LIST_HEAD = object()

T = TypeVar('T')


@with_slots
@dataclass
class LinkedListNode(Generic[T]):
    """Node of LinkedList."""

    data: T
    next: LinkedListNode[T]
    previous: LinkedListNode[T]
    parent: LinkedList[T]

    @staticmethod
    def head(parent: LinkedList) -> LinkedListNode:
        """Create a head node for a linked list."""
        head = LinkedListNode(LINKED_LIST_HEAD, None, None, parent)
        head.next = head.previous = head
        return head

    @property
    def is_head(self) -> bool:
        """Get whether node is head of the list."""
        return self.data is LINKED_LIST_HEAD

    @property
    def has_next(self) -> bool:
        """Check if there is a node after this one."""
        return self.next.data is not LINKED_LIST_HEAD

    @property
    def has_previous(self) -> bool:
        """Check if there is a node before this one."""
        return self.previous.data is not LINKED_LIST_HEAD

    def remove(self):
        """Remove this node from the list."""
        self.parent.remove_node(self)


class LinkedList(MutableSequence, Generic[T]):
    """Doubly linked list implementation.

    Implements all MutableSequence and collections.deque methods. In addition
    to those, allows direct access to the linked list nodes, returning the
    nodes in insertion operations and with the `node` or `index_and_node`
    methods. From the nodes, it's possible to navigate using the `next` and
    `previous` attributes.
    """

    __slots__ = '_head', '_len'

    _head: LinkedListNode[T]
    _len: int

    def __init__(self, values: Iterable[T] = None):
        self._len = 0
        self._head = LinkedListNode.head(self)
        if values:
            self.extend(values)

    def node(self, index: int) -> LinkedListNode[T]:
        """Get the node in position `index`."""
        if not -self._len <= index < self._len:
            raise IndexError('Index out of range.')

        if index < 0:
            index += self._len

        if index < self._len / 2:
            node = self._head.next
            for _ in range(index):
                node = node.next
        else:
            node = self._head.previous
            for _ in range(self._len - index - 1):
                node = node.previous

        return node

    def remove_node(self, node: LinkedListNode[T]):
        """Remove given node from list."""
        if node.parent is not self:
            raise ValueError('Node is not from this list.')

        node.previous.next = node.next
        node.next.previous = node.previous
        self._len -= 1

    def _insert_node(self, new_node: LinkedListNode[T],
                     reference_node: LinkedListNode[T]):
        new_node.next = reference_node
        new_node.previous = reference_node.previous
        reference_node.previous.next = new_node
        reference_node.previous = new_node
        self._len += 1

    def index_and_node(self, value: T, start: int = 0, stop: int = None) \
            -> Tuple[int, LinkedListNode[T]]:
        """Get first index of value.

        Raises ValueError if the value is not present.
        """
        if start is not None and start < 0:
            start = max(self._len + start, 0)
        if stop is not None and stop < 0:
            stop += self._len
        elif stop is None:
            stop = self._len

        node = self._head.next
        for _ in range(start):
            node = node.next

        for i in range(start, stop):
            if node.data is value or node.data == value:
                return i, node
            node = node.next

        raise ValueError('Value not found.')

    def index(self, value: T, start: int = 0, stop: int = None) -> int:
        """Get first index of value.

        Raises ValueError if the value is not present.
        """
        return self.index_and_node(value, start, stop)[0]

    def insert(self, index: int, value: T) -> LinkedListNode[T]:
        """Insert value before index."""
        node = LinkedListNode(value, None, None, self)

        if index == 0:
            reference_node = self._head.next
        elif index == self._len:
            reference_node = self._head
        else:
            reference_node = self.node(index)

        self._insert_node(node, reference_node)
        return node

    def append(self, value: T) -> LinkedListNode[T]:
        """Append value to the end of the list."""
        return self.insert(self._len, value)

    def appendleft(self, value: T) -> LinkedListNode[T]:
        """Append value to the start of the list."""
        return self.insert(0, value)

    def clear(self):
        """Remove all items from the list."""
        self._head.next = self._head.previous = self._head
        self._len = 0

    def reverse(self):
        """Reverse list in place."""
        node = self._head
        next_ = None
        while next_ is not self._head:
            next_ = node.next
            node.next, node.previous = node.previous, node.next
            node = next_

    def extend(self, values: Iterable[T]):
        """Extend list by appending elements from the iterable."""
        for value in values:
            self.insert(self._len, value)

    def extendleft(self, values: Iterable[T]):
        """Extend the list start by appending elements from iterable.

        Note, the series of left appends results in reversing the order of
        elements in the `values` argument.
        """
        for value in values:
            self.insert(0, value)

    def pop(self, index: int = -1) -> T:
        """Remove and return item at index (default last).

        Raise IndexError if list is empty or index is out of range.
        """
        node = self.node(index)
        value = node.data
        self.remove_node(node)
        return value

    def popleft(self) -> T:
        """Remove and return the first item.

        Raise IndexError if list is empty.
        """
        node = self.node(0)
        value = node.data
        self.remove_node(node)
        return value

    def remove(self, value: T):
        """Remove first occurrence of value.

        Raise ValueError if the value is not present.
        """
        _, node = self.index_and_node(value)
        self.remove_node(node)

    def rotate(self, steps: int = 1):
        """Rotate the list `steps` to the right.

        If `steps` is negative, rotate to the left. When the list is not empty,
        rotating one step to the right is equivalent to l.appendleft(l.pop()),
        and rotating one step to the left is equivalent to
        l.append(l.popleft()).
        """
        steps = int(steps) % self._len
        if self._len == 0 or steps == 0:
            return

        reference_node = self.node(self._len - steps)
        self._head.previous.next = self._head.next
        self._head.next.previous = self._head.previous
        self._head.next = reference_node
        self._head.previous = reference_node.previous
        reference_node.previous.next = self._head
        reference_node.previous = self._head

    def __len__(self):
        return self._len

    def __getitem__(self, index):
        return self.node(index).data

    def __setitem__(self, index, value):
        self.node(index).data = value

    def __delitem__(self, index):
        self.remove_node(self.node(index))

    def __iter__(self):
        node = self._head.next
        while node is not self._head:
            yield node.data
            node = node.next

    def __reversed__(self):
        node = self._head.previous
        while node is not self._head:
            yield node.data
            node = node.previous

    def __repr__(self):
        return f'{LinkedList.__name__}([{", ".join(str(i) for i in self)}])'

    def __str__(self):
        return f'[{", ".join(str(i) for i in self)}]'

    def __getstate__(self):
        return list(self)

    def __setstate__(self, state):
        self.__init__(state)
