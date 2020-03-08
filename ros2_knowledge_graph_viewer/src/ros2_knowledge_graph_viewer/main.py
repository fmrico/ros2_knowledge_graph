#!/usr/bin/env python3

import sys

from rqt_gui.main import Main


def main():
    sys.exit(Main().main(sys.argv, standalone='ros2_knowledge_graph_viewer.ros2_knowledge_graph.Ros2KnowledgeGraph'))


if __name__ == '__main__':
    main()
